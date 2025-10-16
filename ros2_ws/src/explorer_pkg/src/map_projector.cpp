#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <cmath>
#include <chrono>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h> 
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MapProjector : public rclcpp::Node
{
public:
    MapProjector() : Node("map_projector")
    {
        this->declare_parameter("z_min", -0.5);
        this->declare_parameter("z_max", 2.0);
        this->declare_parameter("obstacle_thresh", 5);
        this->declare_parameter("known_thresh", 3);
        this->declare_parameter("resolution", 0.1);
        this->declare_parameter("width", 50.0);
        this->declare_parameter("height", 50.0);
        //this->declare_parameter("cloud_topic", "/cloud_registered");
        this->declare_parameter("l1_cloud_topic", "/utlidar/cloud");
        this->declare_parameter("lio_sam_cloud_topic", "/lio_sam_ros2/mapping/cloud_registered");
        this->declare_parameter("l1_cloud_skip", 1);
        this->declare_parameter("lio_sam_cloud_skip", 1);
        this->declare_parameter("map_topic", "/projected_map");
        this->declare_parameter("publish_interval", 2.0); 


        z_min_ = this->get_parameter("z_min").as_double();
        z_max_ = this->get_parameter("z_max").as_double();
        obstacle_thresh_ = this->get_parameter("obstacle_thresh").as_int();
        known_thresh_ = this->get_parameter("known_thresh").as_int();
        resolution_ = this->get_parameter("resolution").as_double();
        width_ = this->get_parameter("width").as_double();
        height_ = this->get_parameter("height").as_double();
        std::string l1_cloud_topic = this->get_parameter("l1_cloud_topic").as_string();
        std::string lio_sam_cloud_topic = this->get_parameter("lio_sam_cloud_topic").as_string();
        std::string map_topic = this->get_parameter("map_topic").as_string();
        publish_interval_ = this->get_parameter("publish_interval").as_double();
        l1_cloud_skip_ = this->get_parameter("l1_cloud_skip").as_int();
        lio_sam_cloud_skip_ = this->get_parameter("lio_sam_cloud_skip").as_int();

        grid_width_ = static_cast<int>(std::ceil(width_ / resolution_));
        grid_height_ = static_cast<int>(std::ceil(height_ / resolution_));
        total_cells_ = grid_width_ * grid_height_;

        origin_x_ = -width_ / 2.0;
        origin_y_ = -height_ / 2.0;

        // Preallocazione degli array di contatori per efficienza
        obstacle_count_.resize(total_cells_, 0);
        floor_count_.resize(total_cells_, 0);

        occupancy_grid.header.stamp = this->get_clock()->now();
        occupancy_grid.header.frame_id = "map";

        occupancy_grid.info.resolution = resolution_;
        occupancy_grid.info.width = grid_width_;
        occupancy_grid.info.height = grid_height_;
        occupancy_grid.info.origin.position.x = origin_x_;
        occupancy_grid.info.origin.position.y = origin_y_;
        occupancy_grid.info.origin.position.z = 0.0;
        occupancy_grid.info.origin.orientation.w = 1.0;
        occupancy_grid.data.resize(total_cells_, -1);

        // Creazione dei subscriber per PointCloud2
        /*
                l1_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            l1_cloud_topic, 
            rclcpp::QoS(1).reliable(),
            std::bind(&MapProjector::pointcloudCallback, this, std::placeholders::_1));
        lio_sam_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lio_sam_cloud_topic, 
            rclcpp::QoS(1).reliable(),
            std::bind(&MapProjector::pointcloudCallback, this, std::placeholders::_1));
  
        */
        l1_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            l1_cloud_topic, 
            rclcpp::QoS(1).reliable(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                static int l1_counter = 0;
                if (l1_counter % l1_cloud_skip_ == 0) {
                    this->pointcloudCallback(msg);
                }
                l1_counter++;
            });
        lio_sam_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lio_sam_cloud_topic, 
            rclcpp::QoS(1).reliable(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                static int lio_sam_counter = 0;
                if (lio_sam_counter % lio_sam_cloud_skip_ == 0) {
                    this->pointcloudCallback(msg);
                }
                lio_sam_counter++;
            });

        occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            map_topic, 
            rclcpp::QoS(1).reliable().transient_local());
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(publish_interval_ * 1000)), // Pubblica ogni 500 ms
            std::bind(&MapProjector::publish_grid_callback, this)); 

        // TF2 
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        RCLCPP_INFO(this->get_logger(), "MapProjector inizializzato con parametri:");
        RCLCPP_INFO(this->get_logger(), "  z_min: %.2f, z_max: %.2f", z_min_, z_max_);
        RCLCPP_INFO(this->get_logger(), "  obstacle_thresh: %d, known_thresh: %d", obstacle_thresh_, known_thresh_);
        RCLCPP_INFO(this->get_logger(), "  resolution: %.2f, width: %.1f, height: %.1f", resolution_, width_, height_);
        RCLCPP_INFO(this->get_logger(), "  Griglia: %dx%d celle, origine: (%.2f, %.2f)", 
                    grid_width_, grid_height_, origin_x_, origin_y_);
    }

private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::string frame_id = msg->header.frame_id;
        auto start = this->now();

        sensor_msgs::msg::PointCloud2 cloud_in = *msg;
        sensor_msgs::msg::PointCloud2 cloud_out;

        geometry_msgs::msg::TransformStamped transform;

        if(frame_id != "map" && frame_id != "odom") {
            try {
                transform = tf_buffer_->lookupTransform(
                    "map",
                    cloud_in.header.frame_id,
                    tf2::TimePointZero
                );

                tf2::doTransform(cloud_in, cloud_out, transform);
            }
            catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
                return;
            }
        }
        else {
            cloud_out = cloud_in;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(cloud_out, cloud);
        for (const auto& point : cloud.points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue;

            // Filtro altezza: ignora punti troppo alti
            if (point.z > z_max_)
                continue;

            int grid_x = static_cast<int>((point.x - origin_x_) / resolution_);
            int grid_y = static_cast<int>((point.y - origin_y_) / resolution_);

            if (grid_x < 0 || grid_x >= grid_width_ || grid_y < 0 || grid_y >= grid_height_)
                continue;

            int cell_index = grid_y * grid_width_ + grid_x;

            if (point.z >= z_min_ && point.z <= z_max_)
            {
                // Punto ostacolo
                obstacle_count_[cell_index]++;
            }
            else if (point.z < z_min_)
            {
                // Punto pavimento
                floor_count_[cell_index]++;
            }
        }
        auto end = this->now();
        auto duration = end - start;
        RCLCPP_INFO(this->get_logger(), "PointCloud da frame %s processata in %.2f ms, punti: %d", 
                    frame_id.c_str(), duration.seconds() * 1000.0, (int)cloud.points.size());

    }
    void publish_grid_callback()
    {
        auto start = this->now();

        for (int i = 0; i < total_cells_; ++i)
        {
            if (obstacle_count_[i] == 0 && floor_count_[i] == 0) continue;
            if (obstacle_count_[i] >= obstacle_thresh_)
            {
                // Cella occupata
                occupancy_grid.data[i] = 100;
                //occupied_cells++;
            }
            else if (obstacle_count_[i] < obstacle_thresh_ && floor_count_[i] >= known_thresh_)
            {
                // Cella libera
                occupancy_grid.data[i] = 0;
                //free_cells++;
            }
        }

        occupancy_grid_pub_->publish(occupancy_grid);
        std::fill(obstacle_count_.begin(), obstacle_count_.end(), 0);
        std::fill(floor_count_.begin(), floor_count_.end(), 0);
        auto end = this->now();
        auto duration = end - start;
        RCLCPP_INFO(this->get_logger(), "OccupancyGrid pubblicata in %.2f ms", duration.seconds() * 1000.0);

    }

    double z_min_, z_max_;
    int obstacle_thresh_, known_thresh_;
    double resolution_, width_, height_;
    double publish_interval_; // intervallo di pubblicazione in secondi

    int grid_width_, grid_height_, total_cells_;
    double origin_x_, origin_y_;

    std::vector<int> obstacle_count_;
    std::vector<int> floor_count_;

    // ROS2 publishers e subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr l1_pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lio_sam_pointcloud_sub_;
    int l1_cloud_skip_;
    int lio_sam_cloud_skip_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    nav_msgs::msg::OccupancyGrid occupancy_grid;

    rclcpp::TimerBase::SharedPtr timer_;

    

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MapProjector>();
    
    RCLCPP_INFO(node->get_logger(), "Avvio MapProjector node...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
