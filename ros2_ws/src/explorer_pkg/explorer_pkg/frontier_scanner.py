#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Point, PoseStamped, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
import numpy as np
import random
import math
from typing import List, Tuple, Optional
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import time
from explorer_pkg.dbscan import DBScan
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose

class FrontierScanner(Node):
    def __init__(self):
        super().__init__('frontier_scanner')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_topic', '/map'),
                ('costmap_topic', '/global_costmap/costmap'),
                ('publish_frontiers_topic', '/frontiers'),
                ('publish_candidates_topic', '/frontier_candidates'),
                ('publish_scores_topic', '/frontier_scores'),
                ('frequency', 1.0),
                ('path_service', '/planner_server/get_plan'),
                ('connectivity', '8'),
                ('reduction', 'none'),
                ('dbscan_eps', 1.0),
                ('dbscan_min_pts', 3),
                ('stride_step', 2),
                ('n_random', 10),
                ('metric', 'euclidean'),
                ('robot_frame', 'base_footprint'),
                ('global_frame', 'map'),
                ('occupancy_threshold', 50),
                ('unknown_value', -1),
                ('info_gain_radius', 5),
                ('medoids_choice', 'random')
            ]
        )
        
        self._load_parameters()
        
        self.map_data = None
        self.costmap_data = None
        self.robot_pose = None
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )
        
        self.costmap_subscriber = self.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self.costmap_callback,
            10
        )
        
        self.frontiers_publisher = self.create_publisher(
            PointCloud2,
            self.publish_frontiers_topic,
            10
        )
        
        self.candidates_publisher = self.create_publisher(
            MarkerArray,
            self.publish_candidates_topic,
            10
        )
        
        self.scores_publisher = self.create_publisher(
            Float32MultiArray,
            self.publish_scores_topic,
            10
        )
        
        if self.metric == 'path_length':
            self.path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        
        self.timer = self.create_timer(
            1.0 / self.frequency,
            self.timer_callback
        )
        
        self.dbscan = DBScan(eps=self.dbscan_eps, min_pts=self.dbscan_min_pts)
        
        self.get_logger().info(f'FrontierScanner initialized with frequency {self.frequency} Hz')
    
    def _load_parameters(self):
        """Carica tutti i parametri dal file YAML"""
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        self.publish_frontiers_topic = self.get_parameter('publish_frontiers_topic').get_parameter_value().string_value
        self.publish_candidates_topic = self.get_parameter('publish_candidates_topic').get_parameter_value().string_value
        self.publish_scores_topic = self.get_parameter('publish_scores_topic').get_parameter_value().string_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.path_service = self.get_parameter('path_service').get_parameter_value().string_value
        self.connectivity = self.get_parameter('connectivity').get_parameter_value().string_value
        self.reduction = self.get_parameter('reduction').get_parameter_value().string_value
        self.dbscan_eps = self.get_parameter('dbscan_eps').get_parameter_value().double_value
        self.dbscan_min_pts = self.get_parameter('dbscan_min_pts').get_parameter_value().integer_value
        self.stride_step = self.get_parameter('stride_step').get_parameter_value().integer_value
        self.n_random = self.get_parameter('n_random').get_parameter_value().integer_value
        self.metric = self.get_parameter('metric').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.occupancy_threshold = self.get_parameter('occupancy_threshold').get_parameter_value().integer_value
        self.unknown_value = self.get_parameter('unknown_value').get_parameter_value().integer_value
        self.info_gain_radius = self.get_parameter('info_gain_radius').get_parameter_value().integer_value 
        self.medoids_choice = self.get_parameter('medoids_choice').get_parameter_value().string_value
    
    def map_callback(self, msg: OccupancyGrid):
        """Callback per la mappa principale"""
        self.map_data = msg
    
    def costmap_callback(self, msg: OccupancyGrid):
        """Callback per la costmap"""
        self.costmap_data = msg
    
    def get_robot_pose(self) -> Optional[Tuple[float, float]]:
        """Ottiene la posizione corrente del robot"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return (x, y)
        except TransformException as e:
            self.get_logger().warn(f'Failed to get robot pose: {e}')
            return None
    
    def timer_callback(self):
        """Callback principale del timer"""
        start_time = time.time()
        self._load_parameters()

        if self.map_data is None:
            self.get_logger().info('No map data available')
            return
        
        self.robot_pose = self.get_robot_pose()
        if self.robot_pose is None:
            self.get_logger().info('No robot pose available')
            return
        
        frontiers = self.compute_frontiers()
        if not frontiers:
            self.get_logger().info('No frontiers found')
            # self.publish_empty_results()
            return
        
        candidates = self.apply_reduction(frontiers)
        if not candidates:
            self.get_logger().info(f'Published {len(frontiers)} frontiers, no candidates after reduction')
            # self.publish_empty_results()
            return
        
        scores = self.compute_metrics(candidates)
        scored_candidates = list(zip(candidates, scores))
        scored_candidates.sort(key=lambda x: x[1], reverse=True)
        candidates, scores = zip(*scored_candidates)
        candidates = list(candidates)
        scores = list(scores)
        self.publish_results(frontiers, candidates, scores)
        end_time = time.time()
        self.get_logger().info(f'Published {len(frontiers)} frontiers, {len(candidates)} candidates in {end_time - start_time:.4f} s')   

    def compute_frontiers(self) -> List[Tuple[int, int]]:
        """Calcola le celle di frontiera"""
        if self.map_data is None:
            return []
        
        map_array = np.array(self.map_data.data).reshape(
            self.map_data.info.height, self.map_data.info.width
        )
        
        frontiers = []
        height, width = map_array.shape
        
        if self.connectivity == '4':
            neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        else:  # 8-connessione
            neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), 
                        (0, 1), (1, -1), (1, 0), (1, 1)]
        
        for i in range(height):
            for j in range(width):
                if map_array[i, j] != 0:  # 0 = libera
                    continue
                
                is_frontier = False
                for di, dj in neighbors:
                    ni, nj = i + di, j + dj
                    if 0 <= ni < height and 0 <= nj < width:
                        if map_array[ni, nj] == self.unknown_value:  # -1 = sconosciuta
                            is_frontier = True
                            break
                
                if is_frontier:
                    if self.is_navigable_in_costmap(i, j):
                        frontiers.append((i, j))
        
        return frontiers
    
    def is_navigable_in_costmap(self, row: int, col: int) -> bool:
        """Verifica se una cella è navigabile nella costmap"""
        if self.costmap_data is None:
            return True  # Se non abbiamo costmap, assumiamo navigabile
        
        map_x, map_y = self.grid_to_world(col, row, self.map_data)
        cost_col, cost_row = self.world_to_grid(map_x, map_y, self.costmap_data)
        
        if (0 <= cost_row < self.costmap_data.info.height and 
            0 <= cost_col < self.costmap_data.info.width):
            
            cost_array = np.array(self.costmap_data.data).reshape(
                self.costmap_data.info.height, self.costmap_data.info.width
            )
            return cost_array[cost_row, cost_col] < self.occupancy_threshold
        
        return False
    
    def grid_to_world(self, col: int, row: int, grid_msg: OccupancyGrid) -> Tuple[float, float]:
        """Converte coordinate griglia in coordinate mondo"""
        x = grid_msg.info.origin.position.x + (col + 0.5) * grid_msg.info.resolution
        y = grid_msg.info.origin.position.y + (row + 0.5) * grid_msg.info.resolution
        return x, y
    
    def world_to_grid(self, x: float, y: float, grid_msg: OccupancyGrid) -> Tuple[int, int]:
        """Converte coordinate mondo in coordinate griglia"""
        col = int((x - grid_msg.info.origin.position.x) / grid_msg.info.resolution)
        row = int((y - grid_msg.info.origin.position.y) / grid_msg.info.resolution)
        return col, row
    
    def apply_reduction(self, frontiers: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Applica la riduzione delle frontiere"""
        if self.reduction == 'none' or not frontiers:
            return frontiers
        
        elif self.reduction == 'random':
            if len(frontiers) <= self.n_random:
                return frontiers
            return random.sample(frontiers, self.n_random)
        
        elif self.reduction == 'stride':
            return [
                (i, j) 
                for (i, j) in frontiers 
                if (i % self.stride_step == 0 and j % self.stride_step == 0)
            ]
        
        elif self.reduction == 'dbscan':
            self.dbscan.eps = self.dbscan_eps
            self.dbscan.min_pts = self.dbscan_min_pts
            representatives = self.dbscan.fit(frontiers) 
            if self.medoids_choice != 'random':
                medoids = self.compute_medoids()   
                if medoids is not None:
                    return medoids
            return representatives
        
        else:
            self.get_logger().warn(f'Unknown reduction method: {self.reduction}')
            return frontiers
    
    def compute_metrics(self, candidates: List[Tuple[int, int]]) -> List[float]:
        """Calcola le metriche per i candidati"""
        if not candidates or self.robot_pose is None:
            return []
        
        scores = []
        robot_x, robot_y = self.robot_pose
        
        for row, col in candidates:
            target_x, target_y = self.grid_to_world(col, row, self.map_data)
            
            
            if self.metric == 'euclidean':
                distance = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)
                score = 1.0 / (1.0 + distance) 
            
            elif self.metric == 'path_length':
                path_length = self.get_path_length(target_x, target_y)
                if path_length is not None:
                    score = 1.0 / (1.0 + path_length)
                else:
                    score = 0.0001  
            
            elif self.metric == 'dim_cluster':
                if self.reduction == 'dbscan':
                    dim_clusters = [len(c) for c in self.dbscan.clusters]
                    score = dim_clusters[candidates.index((row, col))]
                else:
                    self.get_logger().warn('dim_cluster metric requires DBScan reduction')
                    score = 1.0
            
            elif self.metric == 'pseudo_info_gain':
                score = self.compute_information_gain(row, col)
            
            else:
                self.get_logger().warn(f'Unknown metric: {self.metric}')
                score = 1.0
            
            scores.append(score)
        
        return scores
    
    def compute_medoids(self) -> Optional[List[Tuple[int, int]]]:
        """Calcola medoidi"""
        if self.medoids_choice == 'pseudo_centroid':
            medoids = []
            for cluster in self.dbscan.clusters:
                if not cluster:
                    continue
                avg_row = sum([pt[0] for pt in cluster]) / len(cluster)
                avg_col = sum([pt[1] for pt in cluster]) / len(cluster)
                medoid = min(cluster, key=lambda pt: math.sqrt((pt[0] - avg_row)**2 + (pt[1] - avg_col)**2))
                medoids.append(medoid)
            return medoids
        elif self.medoids_choice == 'nearest':
            medoids = []
            robot_x, robot_y = self.robot_pose
            for cluster in self.dbscan.clusters:
                if not cluster:
                    continue
                nearest = min(cluster, key=lambda pt: math.sqrt(
                    (self.grid_to_world(pt[1], pt[0], self.map_data)[0] - robot_x)**2 + 
                    (self.grid_to_world(pt[1], pt[0], self.map_data)[1] - robot_y)**2))
                medoids.append(nearest)
            return medoids
        else:  # warning   
            self.get_logger().warn(f'Unknown medoids choice: {self.medoids_choice}')
            return None







    def get_path_length(self, goal_x: float, goal_y: float) -> Optional[float]:
        """Calcola la lunghezza del path tramite l'action ComputePathToPose"""
        if not hasattr(self, 'path_client'):
            self.path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        if not self.path_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('ComputePathToPose action server not available')
            return None

        goal_msg = ComputePathToPose.Goal()

        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0
        goal_msg.planner_id = ''  # usa quello di default

        future = self.path_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        goal_handle = future.result()
        if goal_handle is None:
            self.get_logger().warn('Failed to get goal handle from action server')
            return None

        if not goal_handle.accepted:
            self.get_logger().warn('Path planning goal was rejected')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=1.0)
        result = result_future.result().result

        if result is None or not result.path.poses:
            self.get_logger().warn('Path planning failed or empty path')
            return None

        path_length = 0.0
        poses = result.path.poses
        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i + 1].pose.position
            path_length += math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)

        return path_length        
    def compute_information_gain(self, row: int, col: int) -> float:
        """Calcola un pseudo-information gain"""
        if self.map_data is None:
            return 1.0
        
        map_array = np.array(self.map_data.data).reshape(
            self.map_data.info.height, self.map_data.info.width
        )
        
        radius = self.info_gain_radius  # celle
        unknown_count = 0
        total_count = 0
        
        for di in range(-radius, radius + 1):
            for dj in range(-radius, radius + 1):
                ni, nj = row + di, col + dj
                if (0 <= ni < map_array.shape[0] and 
                    0 <= nj < map_array.shape[1]):
                    total_count += 1
                    if map_array[ni, nj] == self.unknown_value:
                        unknown_count += 1
        
        if total_count == 0:
            return 0.0
        
        return float(unknown_count) / float(total_count)
    
    def publish_results(self, frontiers: List[Tuple[int, int]], 
                    candidates: List[Tuple[int, int]], 
                    scores: List[float]):
        """Pubblica frontiere e candidati come PointCloud2"""
        current_time = self.get_clock().now().to_msg()

        # -------- FRONTIERE --------
        frontier_points = []
        for row, col in frontiers:
            x, y = self.grid_to_world(col, row, self.map_data)
            frontier_points.append((x, y, 0.0))

        frontiers_msg = PointCloud2()
        frontiers_msg.header = Header(stamp=current_time, frame_id=self.global_frame)
        frontiers_msg.height = 1
        frontiers_msg.width = len(frontier_points)
        frontiers_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        frontiers_msg.is_bigendian = False
        frontiers_msg.point_step = 12  # 3 * 4 byte
        frontiers_msg.row_step = frontiers_msg.point_step * frontiers_msg.width
        frontiers_msg.is_dense = True
        frontiers_msg.data = b"".join([struct.pack('fff', x, y, z) for x, y, z in frontier_points])
        self.frontiers_publisher.publish(frontiers_msg)

        # -------- CANDIDATI --------
        candidates_marker_array = MarkerArray()
        scores_scaled = [s / max(scores) for s in scores]

        for idx, ((row, col), score) in enumerate(zip(candidates, scores_scaled)):
            x, y = self.grid_to_world(col, row, self.map_data)

            marker = Marker()
            marker.header.frame_id = self.global_frame
            marker.header.stamp = current_time
            marker.ns = "candidates"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # posizione
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            # dimensione visibile
            marker.scale.x = 0.07
            marker.scale.y = 0.07
            marker.scale.z = 0.07

            # colore dal punteggio (0=rosso, 1=verde)
            marker.color.r = 1.0 - score
            marker.color.g = score
            marker.color.b = 0.0
            marker.color.a = 1.0

            candidates_marker_array.markers.append(marker)

        self.candidates_publisher.publish(candidates_marker_array)
        # -------- SCORES --------
        scores_msg = Float32MultiArray()
        scores_msg.data = [float(score) for score in scores]
        self.scores_publisher.publish(scores_msg)

        # self.get_logger().debug(f'Published {len(frontiers)} frontiers, {len(candidates)} candidates')



def main(args=None):
    rclpy.init(args=args)
    node = FrontierScanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


