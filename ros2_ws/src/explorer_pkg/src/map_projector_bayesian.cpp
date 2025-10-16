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
        // Dichiarazione dei parametri con valori di default
        this->declare_parameter("z_min", -0.5);
        this->declare_parameter("z_max", 2.0);
        this->declare_parameter("obstacle_thresh", 5);
        //this->declare_parameter("known_thresh", 3);
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
        // Log-odds parameters, ros parameters sono probabilità non log odds
        this->declare_parameter("prob_pos_tresh", 0.7); // Probabilità per cella occupata
        this->declare_parameter("prob_neg_tresh", 0.3); // Probabilità per cella libera
        this->declare_parameter("log_odds_max_incr", 0.9); // Massimo incremento di log-odds per aggiornamento
        this->declare_parameter("log_odds_min_incr", 0.4); // Minimo




        // Lettura dei parametri
        z_min_ = this->get_parameter("z_min").as_double();
        z_max_ = this->get_parameter("z_max").as_double();
        obstacle_thresh_ = this->get_parameter("obstacle_thresh").as_int();
        //known_thresh_ = this->get_parameter("known_thresh").as_int();
        resolution_ = this->get_parameter("resolution").as_double();
        width_ = this->get_parameter("width").as_double();
        height_ = this->get_parameter("height").as_double();
        std::string l1_cloud_topic = this->get_parameter("l1_cloud_topic").as_string();
        std::string lio_sam_cloud_topic = this->get_parameter("lio_sam_cloud_topic").as_string();
        std::string map_topic = this->get_parameter("map_topic").as_string();
        publish_interval_ = this->get_parameter("publish_interval").as_double();
        l1_cloud_skip_ = this->get_parameter("l1_cloud_skip").as_int();
        lio_sam_cloud_skip_ = this->get_parameter("lio_sam_cloud_skip").as_int();
        // Log-odds parameters
        log_odds_max_incr_ = this->get_parameter("log_odds_max_incr").as_double();
        log_odds_min_incr_ = this->get_parameter("log_odds_min_incr").as_double();
        log_odds_pos_thresh_ = prob_to_log_odds(this->get_parameter("prob_pos_tresh").as_double());
        log_odds_neg_thresh_ = prob_to_log_odds(this->get_parameter("prob_neg_tresh").as_double());


        // Calcolo delle dimensioni della griglia in celle
        grid_width_ = static_cast<int>(std::ceil(width_ / resolution_));
        grid_height_ = static_cast<int>(std::ceil(height_ / resolution_));
        total_cells_ = grid_width_ * grid_height_;

        // Calcolo dell'origine della griglia (angolo in basso a sinistra)
        // Il frame map è al centro della griglia
        origin_x_ = -width_ / 2.0;
        origin_y_ = -height_ / 2.0;

        // Preallocazione degli array di contatori per efficienza
        obstacle_count_.resize(total_cells_, 0);
        floor_count_.resize(total_cells_, 0);

        // Inizializzazione della mappa di occupazione, all'inizio tutti sconosciuti
        // Configurazione header
        occupancy_grid.header.stamp = this->get_clock()->now();
        occupancy_grid.header.frame_id = "map";

        // Configurazione info della griglia
        occupancy_grid.info.resolution = resolution_;
        occupancy_grid.info.width = grid_width_;
        occupancy_grid.info.height = grid_height_;
        occupancy_grid.info.origin.position.x = origin_x_;
        occupancy_grid.info.origin.position.y = origin_y_;
        occupancy_grid.info.origin.position.z = 0.0;
        occupancy_grid.info.origin.orientation.w = 1.0;

        // Preallocazione del vettore dati
        occupancy_grid.data.resize(total_cells_, -1);
        log_odds_map_.resize(total_cells_, 0.0);


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
        // Subscriber L1 con lambda per skip
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
        // Subscriber LIO-SAM con lambda per skip
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

        // Creazione del publisher per OccupancyGrid
        occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            map_topic, 
            rclcpp::QoS(1).reliable().transient_local());
        // Creazione del timer per pubblicare la mappa a intervalli regolari, con una publish_grid_callback già definita, non usare [this]
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(publish_interval_ * 1000)), // Pubblica ogni 500 ms
            std::bind(&MapProjector::publish_grid_callback, this)); 

        // TF2 
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        

        // Log dei parametri caricati
        RCLCPP_INFO(this->get_logger(), "MapProjector inizializzato con parametri:");
        RCLCPP_INFO(this->get_logger(), "  z_min: %.2f, z_max: %.2f", z_min_, z_max_);
        RCLCPP_INFO(this->get_logger(), "  obstacle_thresh: %d", obstacle_thresh_);
        RCLCPP_INFO(this->get_logger(), "  resolution: %.2f, width: %.1f, height: %.1f", resolution_, width_, height_);
        RCLCPP_INFO(this->get_logger(), "  Griglia: %dx%d celle, origine: (%.2f, %.2f)", 
                    grid_width_, grid_height_, origin_x_, origin_y_);
    }

private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Cronometra callback
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
        // Loop principale sui punti del cloud per aggiornare i contatori
        for (const auto& point : cloud.points)
        {
            // Skip punti invalidi
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue;

            // Filtro altezza: ignora punti troppo alti
            if (point.z > z_max_)
                continue;

            // Calcolo indici nella griglia
            int grid_x = static_cast<int>((point.x - origin_x_) / resolution_);
            int grid_y = static_cast<int>((point.y - origin_y_) / resolution_);

            // Verifica che il punto sia dentro la griglia
            if (grid_x < 0 || grid_x >= grid_width_ || grid_y < 0 || grid_y >= grid_height_)
                continue;

            // Calcolo indice lineare (row-major order per OccupancyGrid)
            int cell_index = grid_y * grid_width_ + grid_x;

            // Classificazione del punto e aggiornamento contatori
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
        // Log statistiche con frame_id e tempo e numero di punti
        RCLCPP_INFO(this->get_logger(), "PointCloud da frame %s processata in %.2f ms, punti: %d", 
                    frame_id.c_str(), duration.seconds() * 1000.0, (int)cloud.points.size());

    }
    static inline double prob_to_log_odds(double p) {
        return log(p / (1.0 - p));
    }
    inline double delta_L(int i) const {
        double ret;
        if(obstacle_count_[i] > obstacle_thresh_) ret = log_odds_min_incr_*obstacle_count_[i];
        else ret = -log_odds_min_incr_*floor_count_[i];
        //saturazione
        if(ret > log_odds_max_incr_) return log_odds_max_incr_;
        else if(ret < -log_odds_max_incr_) return -log_odds_max_incr_;
        else return ret;
    }
    void publish_grid_callback()
    {
        // Cronometra callback
        auto start = this->now();
        // Loop finale per classificare le celle e generare OccupancyGrid
        //int occupied_cells = 0, free_cells = 0, unknown_cells = 0;

        for (int i = 0; i < total_cells_; ++i)
        {
            if (obstacle_count_[i] == 0 && floor_count_[i] == 0) continue;
            /*
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
            
            */
            // Aggiornamento log-odds
            log_odds_map_[i] += delta_L(i);
            // Aggiornamento stato cella basato su log-odds
            if(log_odds_map_[i] > log_odds_pos_thresh_) {
                occupancy_grid.data[i] = 100; // Occupata
            }
            else if(log_odds_map_[i] < log_odds_neg_thresh_) {
                occupancy_grid.data[i] = 0; // Libera
            }
            else {
                occupancy_grid.data[i] = -1; // Sconosciuta
            }
        }

        // Pubblicazione del messaggio
        occupancy_grid_pub_->publish(occupancy_grid);
        std::fill(obstacle_count_.begin(), obstacle_count_.end(), 0);
        std::fill(floor_count_.begin(), floor_count_.end(), 0);

        auto end = this->now();
        auto duration = end - start;
        // Log statistiche con tempo
        RCLCPP_INFO(this->get_logger(), "OccupancyGrid pubblicata in %.2f ms", duration.seconds() * 1000.0);

    }

    // Parametri configurabili
    double z_min_, z_max_;
    int obstacle_thresh_;
    double resolution_, width_, height_;
    double publish_interval_; // Intervallo di pubblicazione in secondi

    // Parametri calcolati della griglia
    int grid_width_, grid_height_, total_cells_;
    double origin_x_, origin_y_;

    // Array di contatori preallocati per efficienza
    std::vector<int> obstacle_count_;
    std::vector<int> floor_count_;

    // ROS2 publishers e subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr l1_pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lio_sam_pointcloud_sub_;
    // Skip parameters
    int l1_cloud_skip_;
    int lio_sam_cloud_skip_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Mappa di occupazione
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    // Mappa di log-odds
    std::vector<double> log_odds_map_;
    // Log-odds parameters
    double log_odds_pos_thresh_; // Soglia per cella occupata
    double log_odds_neg_thresh_; // Soglia per cella libera
    double log_odds_max_incr_; // Massimo incremento di log-odds per aggiornamento
    double log_odds_min_incr_; // Minimo

    // Timer per pubblicare periodicamente la occupancy grid con una callback
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
