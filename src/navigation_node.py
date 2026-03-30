#!/usr/bin/env python3
"""
GPS + ACCELEROMETER + CAMERA NAVIGATION FOR USV
No IMU, No Ultrasonic - Uses camera for obstacle avoidance!
"""

import rclpy
from rclpy.node import Node
import math
import numpy as np
from enum import Enum

# Message imports
from usv_autonomy.msg import Debris
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu, Image
from std_msgs.msg import Header, String
from cv_bridge import CvBridge
import cv2

# Try to import geographiclib (optional - for GPS distance calculation)
try:
    from geographiclib.geodesic import Geodesic
    GEO_AVAILABLE = True
except ImportError:
    GEO_AVAILABLE = False
    print("⚠️ geographiclib not installed. Install with: pip3 install geographiclib")

class RobotState(Enum):
    SEARCH = 1
    ALIGN = 2
    APPROACH = 3
    VERIFY = 4
    RETURN_HOME = 5
    AVOID_OBSTACLE = 6  # Now uses camera!

class GPSNavigationNode(Node):
    """
    Navigation using GPS + Accelerometer + Camera (NO Ultrasonic!)
    Camera does obstacle detection and avoidance
    """
    
    def __init__(self):
        super().__init__('navigation_node')
        
        # ============= STATE VARIABLES =============
        self.current_state = RobotState.SEARCH
        self.current_pose = None
        self.current_gps = None
        self.current_velocity = [0.0, 0.0]
        self.current_acceleration = [0.0, 0.0]
        self.obstacle_detected = False  # From camera!
        self.obstacle_direction = 0  # -1 left, 1 right, 0 center
        self.home_position = None
        self.last_gps_time = None
        self.last_accel_time = None
        
        # ============= MISSION HANDLING =============
        self.current_mission = "COVERAGE_SEARCH"
        self.mission_target = None
        
        # ============= COVERAGE PATH =============
        self.coverage_path_gps = []
        self.current_waypoint_index = 0
        self.home_gps = None
        
        # ============= GARBAGE COLLECTION =============
        self.collected_garbage_count = 0
        self.active_garbage_targets = []
        self.ignored_objects = []
        self.garbage_position_memory = None
        self.current_detection = None
        self.verification_received = None
        self.verification_start_time = None
        
        # ============= COLLECTION PARAMETERS =============
        self.forward_drive_speed = 0.25
        self.alignment_threshold = 30
        self.collection_area_threshold = 5000
        
        # ============= NAVIGATION PARAMETERS =============
        self.max_linear_speed = 1.0
        self.max_angular_speed = 1.0
        self.waypoint_tolerance_meters = 2.0
        
        # ============= DEAD RECKONING =============
        self.estimated_x = 0.0
        self.estimated_y = 0.0
        self.estimated_heading = 0.0
        self.velocity_estimate = [0.0, 0.0]
        self.last_gps_x = None
        self.last_gps_y = None
        
        # ============= CAMERA OBSTACLE AVOIDANCE =============
        self.bridge = CvBridge()
        self.obstacle_safety_distance = 100  # pixels - obstacle too close if bounding box > this
        self.avoidance_timer = 0
        self.avoid_duration = 30  # frames to avoid
        
        # ============= PID CONTROLLERS =============
        self.kp_linear = 0.5
        self.kp_angular = 1.0
        
        # ============= SUBSCRIBERS =============
        self.debris_sub = self.create_subscription(
            Debris, '/detections/debris', self.debris_callback, 10)
        
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        
        self.accel_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.accel_callback, 10)
        
        # NEW: Camera for obstacle detection (using your perception node)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_obstacle_callback, 10)
        
        self.mission_sub = self.create_subscription(
            String, '/system/mission_command', self.mission_callback, 10)
        
        # ============= PUBLISHERS =============
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.collection_pub = self.create_publisher(String, '/collection_status', 10)
        self.verification_pub = self.create_publisher(String, '/verification/check', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/estimated_pose', 10)
        
        # Initialize coverage path
        self.initialize_coverage_path()
        
        # Main control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('🎯 GPS + Camera Navigation Node started')
        self.get_logger().info('📡 GPS for absolute positioning')
        self.get_logger().info('📊 Accelerometer for velocity estimation')
        self.get_logger().info('🎥 Camera for obstacle avoidance and fine positioning')

    def initialize_coverage_path(self):
        """Initialize GPS coverage path"""
        if not GEO_AVAILABLE:
            self.get_logger().warn('GPS path planning limited - install geographiclib')
            return
        
        # Default starting coordinates (replace with your actual location)
        start_lat = 37.7749
        start_lon = -122.4194
        
        self.coverage_path_gps = []
        spacing = 10  # meters
        rows = 10
        
        for i in range(rows):
            lat_offset = i * spacing / 111320.0
            current_lat = start_lat + lat_offset
            
            if i % 2 == 0:
                lon_end = start_lon + (spacing * 10) / (111320.0 * math.cos(math.radians(current_lat)))
                self.coverage_path_gps.append((current_lat, start_lon))
                self.coverage_path_gps.append((current_lat, lon_end))
            else:
                lon_end = start_lon - (spacing * 10) / (111320.0 * math.cos(math.radians(current_lat)))
                self.coverage_path_gps.append((current_lat, start_lon))
                self.coverage_path_gps.append((current_lat, lon_end))
        
        self.home_gps = (start_lat, start_lon)
        self.get_logger().info(f'🗺️ Coverage path with {len(self.coverage_path_gps)} waypoints')

    def gps_callback(self, msg):
        """Process GPS data"""
        self.current_gps = (msg.latitude, msg.longitude)
        current_time = self.get_clock().now()
        
        if self.home_gps and GEO_AVAILABLE:
            geod = Geodesic.WGS84
            result = geod.Inverse(self.home_gps[0], self.home_gps[1], 
                                   msg.latitude, msg.longitude)
            self.estimated_x = result['s12'] * math.sin(math.radians(result['azi1']))
            self.estimated_y = result['s12'] * math.cos(math.radians(result['azi1']))
        
        if self.current_pose is None:
            self.current_pose = Pose()
        self.current_pose.position.x = self.estimated_x
        self.current_pose.position.y = self.estimated_y
        
        # Publish pose for visualization
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = self.current_pose
        self.pose_pub.publish(pose_msg)
        
        # Estimate velocity from GPS
        if self.last_gps_time is not None and self.last_gps_x is not None:
            dt = (current_time - self.last_gps_time).nanoseconds / 1e9
            if dt > 0 and dt < 1.0:
                self.velocity_estimate[0] = (self.estimated_x - self.last_gps_x) / dt
                self.velocity_estimate[1] = (self.estimated_y - self.last_gps_y) / dt
                
                # Estimate heading from movement
                if abs(self.velocity_estimate[0]) > 0.01 or abs(self.velocity_estimate[1]) > 0.01:
                    self.estimated_heading = math.atan2(self.velocity_estimate[0], self.velocity_estimate[1])
        
        self.last_gps_time = current_time
        self.last_gps_x = self.estimated_x
        self.last_gps_y = self.estimated_y

    def accel_callback(self, msg):
        """Process accelerometer data"""
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        current_time = self.get_clock().now()
        
        if self.last_accel_time is not None:
            dt = (current_time - self.last_accel_time).nanoseconds / 1e9
            if 0.01 < dt < 0.5:
                # Update velocity from acceleration
                self.velocity_estimate[0] += ax * dt
                self.velocity_estimate[1] += ay * dt
        
        self.last_accel_time = current_time

    def camera_obstacle_callback(self, msg):
        """
        NEW: Camera-based obstacle detection and avoidance
        Uses the perception node's detections to identify obstacles
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width = cv_image.shape[:2]
            
            # Simple obstacle detection using motion/optical flow
            if hasattr(self, 'prev_frame'):
                # Calculate optical flow
                prev_gray = cv2.cvtColor(self.prev_frame, cv2.COLOR_BGR2GRAY)
                curr_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                
                flow = cv2.calcOpticalFlowFarneback(prev_gray, curr_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
                
                # Detect expanding regions (objects getting closer)
                magnitude = np.sqrt(flow[..., 0]**2 + flow[..., 1]**2)
                expanding = magnitude > 5  # Threshold for significant motion
                
                if np.any(expanding):
                    # Find where expansion is happening
                    y_indices, x_indices = np.where(expanding)
                    center_x = np.mean(x_indices)
                    
                    if center_x < width * 0.4:
                        self.obstacle_direction = -1  # Left side
                        self.obstacle_detected = True
                    elif center_x > width * 0.6:
                        self.obstacle_direction = 1  # Right side
                        self.obstacle_detected = True
                    else:
                        self.obstacle_direction = 0  # Center
                        self.obstacle_detected = True
                    
                    self.avoidance_timer = self.avoid_duration
                    self.get_logger().warn(f'🚧 Obstacle detected on {"left" if self.obstacle_direction == -1 else "right" if self.obstacle_direction == 1 else "center"}!')
                else:
                    if self.avoidance_timer <= 0:
                        self.obstacle_detected = False
            
            self.prev_frame = cv_image.copy()
            
        except Exception as e:
            self.get_logger().debug(f'Camera obstacle detection error: {e}')

    def debris_callback(self, msg):
        """Process debris detections"""
        self.current_detection = msg
        self.get_logger().info(f'📡 Received: {msg.label} (conf: {msg.confidence:.2f})')
        
        if msg.label == 'garbage' and msg.confidence > 0.25:
            self.garbage_position_memory = {
                'x': msg.bbox_center_x,
                'y': msg.bbox_center_y,
                'area': msg.bbox_area,
                'timestamp': self.get_clock().now()
            }
            
            garbage_id = f"{msg.bbox_center_x:.1f}_{msg.bbox_center_y:.1f}"
            
            if garbage_id not in self.active_garbage_targets and garbage_id not in self.ignored_objects:
                self.active_garbage_targets.append(garbage_id)
                
                if (self.current_state == RobotState.SEARCH and 
                    self.current_mission in ["COVERAGE_SEARCH", "APPROACH_DEBRIS"]):
                    
                    self.current_state = RobotState.ALIGN
                    self.get_logger().info(f'🎯 ALIGNING with GARBAGE')
        
        elif msg.label in ['plants', 'aquatic_animal']:
            object_id = f"{msg.bbox_center_x:.1f}_{msg.bbox_center_y:.1f}"
            if object_id not in self.ignored_objects:
                self.ignored_objects.append(object_id)
                self.get_logger().info(f'🌿 Ignoring {msg.label}')

    def mission_callback(self, msg):
        """Receive mission commands"""
        self.get_logger().info(f'📨 Mission: {msg.data}')
        
        if ':' in msg.data:
            parts = msg.data.split(':')
            self.current_mission = parts[0]
        else:
            self.current_mission = msg.data
        
        if self.current_mission == "COVERAGE_SEARCH":
            self.current_state = RobotState.SEARCH
        elif self.current_mission == "RETURN_TO_HOME":
            self.current_state = RobotState.RETURN_HOME
        elif self.current_mission == "STANDBY":
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)

    def calculate_distance_to_gps(self, target_lat, target_lon):
        """Calculate distance to GPS waypoint"""
        if self.current_gps is None or not GEO_AVAILABLE:
            return float('inf')
        
        geod = Geodesic.WGS84
        result = geod.Inverse(self.current_gps[0], self.current_gps[1], target_lat, target_lon)
        return result['s12']

    def calculate_control_from_gps(self, target_lat, target_lon):
        """Calculate velocity from GPS waypoint"""
        if self.current_gps is None:
            return 0.0, 0.0
        
        distance = self.calculate_distance_to_gps(target_lat, target_lon)
        
        if distance < self.waypoint_tolerance_meters:
            return 0.0, 0.0
        
        # Simple proportional control
        linear_vel = min(self.max_linear_speed, distance * 0.3)
        
        # Calculate bearing (simplified - use heading from GPS movement)
        angular_vel = 0.0
        
        return linear_vel, angular_vel

    def verification_callback(self, msg):
        """Receive verification result"""
        self.verification_received = (msg.data == "SUCCESS")
        self.get_logger().info(f'📋 Verification: {msg.data}')

    def control_loop(self):
        """Main control loop with camera obstacle avoidance"""
        cmd_vel = Twist()
        v, w = 0.0, 0.0
        
        if self.current_mission == "STANDBY":
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            return
        
        # ============= PRIORITY: OBSTACLE AVOIDANCE (Camera-based) =============
        if self.obstacle_detected and self.avoidance_timer > 0:
            self.avoidance_timer -= 1
            
            if self.obstacle_direction == -1:  # Obstacle on left
                w = self.max_angular_speed * 0.5  # Turn right
                v = -0.2  # Back up slightly
                self.get_logger().debug('🚫 Avoiding obstacle on LEFT - turning RIGHT')
            elif self.obstacle_direction == 1:  # Obstacle on right
                w = -self.max_angular_speed * 0.5  # Turn left
                v = -0.2
                self.get_logger().debug('🚫 Avoiding obstacle on RIGHT - turning LEFT')
            else:  # Obstacle in center
                w = self.max_angular_speed * 0.3  # Turn random direction
                v = -0.3  # Back up
                self.get_logger().debug('🚫 Obstacle in CENTER - backing up')
            
            cmd_vel.linear.x = v
            cmd_vel.angular.z = w
            self.cmd_vel_pub.publish(cmd_vel)
            return
        
        # ============= NORMAL NAVIGATION STATES =============
        
        # SEARCH STATE - GPS Waypoint Following
        if self.current_state == RobotState.SEARCH:
            if self.coverage_path_gps and self.current_waypoint_index < len(self.coverage_path_gps):
                target_lat, target_lon = self.coverage_path_gps[self.current_waypoint_index]
                distance = self.calculate_distance_to_gps(target_lat, target_lon)
                
                if distance < self.waypoint_tolerance_meters:
                    self.current_waypoint_index += 1
                    self.get_logger().info(f'📍 Waypoint {self.current_waypoint_index}/{len(self.coverage_path_gps)}')
                else:
                    v, w = self.calculate_control_from_gps(target_lat, target_lon)
        
        # ALIGN STATE - Camera-based alignment
        elif self.current_state == RobotState.ALIGN:
            if self.current_detection and self.current_detection.label == 'garbage':
                pixel_offset = self.current_detection.bbox_center_x - 320
                
                if abs(pixel_offset) > self.alignment_threshold:
                    w = self.max_angular_speed * (pixel_offset / 320.0) * 0.5
                    v = 0.1
                else:
                    self.current_state = RobotState.APPROACH
                    self.get_logger().info('✅ Aligned - Approaching')
            else:
                self.current_state = RobotState.SEARCH
        
        # APPROACH STATE - Drive through
        elif self.current_state == RobotState.APPROACH:
            if self.current_detection and self.current_detection.label == 'garbage':
                if self.current_detection.bbox_area > self.collection_area_threshold:
                    self.current_state = RobotState.VERIFY
                    verify_msg = String()
                    verify_msg.data = "CHECK_COLLECTION"
                    self.verification_pub.publish(verify_msg)
                else:
                    # Align while approaching
                    pixel_offset = self.current_detection.bbox_center_x - 320
                    w = self.max_angular_speed * (pixel_offset / 320.0) * 0.3
                    v = self.forward_drive_speed
            else:
                self.current_state = RobotState.VERIFY
                verify_msg = String()
                verify_msg.data = "CHECK_COLLECTION"
                self.verification_pub.publish(verify_msg)
        
        # VERIFY STATE
        elif self.current_state == RobotState.VERIFY:
            v, w = 0.0, 0.0
            
            if self.verification_start_time is None:
                self.verification_start_time = self.get_clock().now()
                self.get_logger().info('🔍 Verifying collection...')
            
            if self.verification_received is not None:
                if self.verification_received:
                    self.collected_garbage_count += 1
                    collection_msg = String()
                    collection_msg.data = f"GARBAGE_COLLECTED:{self.collected_garbage_count}"
                    self.collection_pub.publish(collection_msg)
                    self.get_logger().info(f'✅ GARBAGE COLLECTED! Total: {self.collected_garbage_count}')
                    
                    self.current_state = RobotState.SEARCH
                    self.current_detection = None
                    self.verification_start_time = None
                    self.verification_received = None
                else:
                    self.get_logger().warn('⚠️ Verification failed - retrying')
                    self.current_state = RobotState.ALIGN
                    self.verification_start_time = None
                    self.verification_received = None
            
            elapsed = (self.get_clock().now() - self.verification_start_time).nanoseconds / 1e9
            if elapsed > 10.0:
                self.get_logger().warn('⏰ Verification timeout')
                self.current_state = RobotState.SEARCH
                self.verification_start_time = None
        
        # RETURN_HOME STATE
        elif self.current_state == RobotState.RETURN_HOME:
            if self.home_gps:
                distance = self.calculate_distance_to_gps(self.home_gps[0], self.home_gps[1])
                
                if distance < self.waypoint_tolerance_meters:
                    self.get_logger().info('🏠 Home reached!')
                    v, w = 0.0, 0.0
                    self.current_state = RobotState.SEARCH
                else:
                    v, w = self.calculate_control_from_gps(self.home_gps[0], self.home_gps[1])
        
        # Apply limits
        cmd_vel.linear.x = max(min(v, self.max_linear_speed), -self.max_linear_speed)
        cmd_vel.angular.z = max(min(w, self.max_angular_speed), -self.max_angular_speed)
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Status logging
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
        else:
            self.log_counter = 0
            
        if self.log_counter % 20 == 0:
            status = f"State: {self.current_state.name}, Collected: {self.collected_garbage_count}"
            if self.obstacle_detected:
                status += " 🚧 AVOIDING"
            self.get_logger().info(status)

def main(args=None):
    rclpy.init(args=args)
    nav_node = GPSNavigationNode()
    
    # Create verification subscriber
    nav_node.verification_sub = nav_node.create_subscription(
        String, '/verification/result', nav_node.verification_callback, 10)
    
    try:
        rclpy.spin(nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()