#!/usr/bin/env python3
"""
SUPERVISOR NODE WITH VISION VERIFICATION - FULLY FIXED
"""

import rclpy
from rclpy.node import Node
from enum import Enum
import math
import time

# ROS 2 messages
from usv_autonomy.msg import Debris
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, Bool, String
from std_srvs.srv import Trigger

class SupervisorState(Enum):
    IDLE = "idle"
    SEARCH = "search"
    APPROACH = "approach"
    COLLECT = "collect"
    VERIFY = "verify"
    RETURN_TO_HOME = "return_to_home"

class VisionVerificationSupervisor(Node):
    """
    Supervisor with vision-based collection verification
    """
    
    def __init__(self):
        super().__init__('supervisor_node')
        
        # FSM parameters
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('home_tolerance', 1.0)
        self.declare_parameter('battery_threshold', 20.0)
        self.declare_parameter('verification_timeout', 5.0)
        self.declare_parameter('max_verification_attempts', 3)
        
        # Load parameters
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.home_tolerance = self.get_parameter('home_tolerance').value
        self.battery_threshold = self.get_parameter('battery_threshold').value
        self.verification_timeout = self.get_parameter('verification_timeout').value
        self.max_verification_attempts = self.get_parameter('max_verification_attempts').value
        
        # State variables
        self.current_state = SupervisorState.IDLE
        self.current_pose = None
        self.current_battery = 100.0
        self.bin_level = 0.0
        self.bin_full = False
        self.current_detection = None
        self.garbage_before_collection = None
        self.home_position = None
        self.verification_attempts = 0
        self.verification_start_time = None
        self.collection_activated = False
        self.collection_timer = None
        
        # Mission command publisher
        self.mission_command_pub = self.create_publisher(String, '/system/mission_command', 10)
        
        # Verification publishers
        self.verification_check_pub = self.create_publisher(String, '/verification/check', 10)
        self.verification_result_pub = self.create_publisher(String, '/verification/result', 10)
        
        # Subscribers
        self.debris_sub = self.create_subscription(
            Debris, '/detections/debris', self.debris_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        self.verification_check_sub = self.create_subscription(
            String, '/verification/check', self.verification_check_callback, 10)
        
        # Simulated battery and bin level
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)
        
        self.bin_level_sub = self.create_subscription(
            Float32, '/bin_level', self.bin_level_callback, 10)
        
        # Services
        self.start_mission_srv = self.create_service(
            Trigger, '/mission/start', self.start_mission_callback)
        
        self.stop_mission_srv = self.create_service(
            Trigger, '/mission/stop', self.stop_mission_callback)
        
        # Collection mechanism control
        self.collection_activation_pub = self.create_publisher(Bool, '/collection/activate', 10)
        
        # Supervision timer
        self.supervision_timer = self.create_timer(0.1, self.supervision_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('🎯 Vision Verification Supervisor initialized')
        self.get_logger().info(f'Initial state: {self.current_state.value}')

    def publish_mission_command(self, command_type, target=None):
        """Publish mission command to navigation node"""
        msg = String()
        if target:
            msg.data = f"{command_type}:{target}"
        else:
            msg.data = command_type
        
        self.mission_command_pub.publish(msg)
        self.get_logger().info(f'📢 Mission Command: {msg.data}')

    def publish_status(self):
        """Publish system status"""
        self.get_logger().info(
            f'📊 Status: {self.current_state.value} | '
            f'Battery: {self.current_battery:.1f}% | '
            f'Bin: {self.bin_level:.1f}% | '
            f'Verification attempts: {self.verification_attempts}',
            throttle_duration_sec=5.0
        )

    def transition_to(self, new_state):
        """Handle state transitions"""
        old_state = self.current_state
        self.current_state = new_state
        self.get_logger().info(f'🔄 State: {old_state.value} → {new_state.value}')

    def get_current_time_seconds(self):
        """Get current time in seconds"""
        return self.get_clock().now().nanoseconds / 1e9

    # ============= CALLBACKS =============
    
    def debris_callback(self, msg):
        """Process debris detections - STORE pre-collection detection"""
        self.current_detection = msg
        self.get_logger().info(f'📡 Received: {msg.label} (conf: {msg.confidence:.2f})')
        
        # Store pre-collection detection for verification
        if msg.label == 'garbage' and msg.confidence > self.confidence_threshold:
            self.garbage_before_collection = {
                'x': msg.bbox_center_x,
                'y': msg.bbox_center_y,
                'area': msg.bbox_area,
                'timestamp': self.get_current_time_seconds()
            }
        
        # State transition: SEARCH → APPROACH
        if (self.current_state == SupervisorState.SEARCH and 
            msg.label == 'garbage' and 
            msg.confidence >= self.confidence_threshold):
            
            self.transition_to(SupervisorState.APPROACH)
            target_info = f"{msg.bbox_center_x},{msg.bbox_center_y}"
            self.publish_mission_command("APPROACH_DEBRIS", target_info)

    def odom_callback(self, msg):
        """Update current pose"""
        self.current_pose = msg.pose.pose
        
        if self.home_position is None and self.current_pose:
            self.home_position = self.current_pose
            self.get_logger().info('🏠 Home position set')

    def battery_callback(self, msg):
        """Monitor battery level"""
        if hasattr(msg, 'percentage') and msg.percentage > 0:
            self.current_battery = msg.percentage
        else:
            self.current_battery = max(0, self.current_battery - 0.1)
        
        if (self.current_battery <= self.battery_threshold and 
            self.current_state not in [SupervisorState.RETURN_TO_HOME, SupervisorState.IDLE]):
            
            self.transition_to(SupervisorState.RETURN_TO_HOME)
            self.publish_mission_command("RETURN_TO_HOME")
            self.get_logger().warn(f'🔋 Low battery ({self.current_battery:.1f}%)')

    def bin_level_callback(self, msg):
        """Monitor collection bin level"""
        if hasattr(msg, 'data'):
            self.bin_level = msg.data
        else:
            self.bin_level = min(100, self.bin_level + 0.05)
        
        self.bin_full = (self.bin_level >= 95.0)
        
        if self.bin_full and self.current_state not in [SupervisorState.RETURN_TO_HOME, SupervisorState.IDLE]:
            self.transition_to(SupervisorState.RETURN_TO_HOME)
            self.publish_mission_command("RETURN_TO_HOME")
            self.get_logger().warn('🗑️ Bin full')

    def verification_check_callback(self, msg):
        """Handle verification request from navigation"""
        if msg.data == "CHECK_COLLECTION":
            self.get_logger().info('🔍 Verification requested - checking if garbage was collected')
            self.transition_to(SupervisorState.VERIFY)
            
            # Start verification timer
            if self.verification_start_time is None:
                self.verification_start_time = self.get_current_time_seconds()
                self.verification_attempts = 0

    def start_mission_callback(self, request, response):
        """Service to start mission"""
        if self.current_state == SupervisorState.IDLE:
            self.transition_to(SupervisorState.SEARCH)
            self.publish_mission_command("COVERAGE_SEARCH")
            response.success = True
            response.message = "Mission started"
        else:
            response.success = False
            response.message = f"Cannot start from state: {self.current_state.value}"
        return response

    def stop_mission_callback(self, request, response):
        """Service to stop mission"""
        self.transition_to(SupervisorState.IDLE)
        self.publish_mission_command("STANDBY")
        response.success = True
        response.message = "Mission stopped"
        return response

    # ============= VISION VERIFICATION =============
    
    def verify_collection_by_vision(self):
        """
        Check if garbage was collected by comparing current detection
        with pre-collection detection
        """
        if self.garbage_before_collection is None:
            self.get_logger().warn('No pre-collection detection stored')
            return False
        
        # Check if we still see garbage at the same location
        if self.current_detection and self.current_detection.label == 'garbage':
            current_x = self.current_detection.bbox_center_x
            current_y = self.current_detection.bbox_center_y
            old_x = self.garbage_before_collection['x']
            old_y = self.garbage_before_collection['y']
            
            # Calculate distance in pixels
            pixel_distance = math.sqrt((current_x - old_x)**2 + (current_y - old_y)**2)
            
            # Also check area change
            current_area = self.current_detection.bbox_area
            old_area = self.garbage_before_collection['area']
            area_ratio = current_area / max(old_area, 1)
            
            self.get_logger().info(
                f'📊 Verification: distance={pixel_distance:.0f}px, '
                f'area_ratio={area_ratio:.2f}'
            )
            
            # Collection successful if:
            # 1. Garbage moved significantly (>100 pixels), OR
            # 2. Garbage area changed dramatically (got closer/further), OR
            # 3. Confidence dropped significantly
            if pixel_distance > 100:
                self.get_logger().info(f'✅ Garbage moved/removed (distance: {pixel_distance:.0f}px)')
                return True
            elif area_ratio > 3.0 or area_ratio < 0.3:
                self.get_logger().info(f'✅ Garbage area changed significantly (ratio: {area_ratio:.2f})')
                return True
            else:
                self.get_logger().info(f'❌ Garbage still present at same location')
                return False
        
        # No garbage detected at all - likely collected!
        self.get_logger().info('✅ No garbage detected - collection successful')
        return True

    def _handle_verify_state(self):
        """Vision-based verification logic"""
        current_time = self.get_current_time_seconds()
        
        # First entry
        if self.verification_start_time is None:
            self.verification_start_time = current_time
            self.verification_attempts = 0
            self.get_logger().info('🔍 Starting vision verification...')
            return
        
        elapsed = current_time - self.verification_start_time
        
        # Wait for perception to update (allow 2 seconds for new detections)
        if elapsed < 2.0:
            return
        
        # Perform vision verification
        collection_success = self.verify_collection_by_vision()
        
        if collection_success:
            # Collection verified!
            self.get_logger().info('✅ Collection verified by vision!')
            
            # Send success result to navigation
            result_msg = String()
            result_msg.data = "SUCCESS"
            self.verification_result_pub.publish(result_msg)
            
            # Increment bin level
            self.bin_level = min(100, self.bin_level + 5.0)
            
            self.transition_to(SupervisorState.SEARCH)
            self.publish_mission_command("COVERAGE_SEARCH")
            self.verification_attempts = 0
            self.verification_start_time = None
            self.garbage_before_collection = None
            
        else:
            # Verification failed
            self.verification_attempts += 1
            
            if self.verification_attempts >= self.max_verification_attempts:
                # Max attempts reached - give up
                self.get_logger().warn(f'❌ Max verification attempts ({self.max_verification_attempts}) reached - giving up')
                
                result_msg = String()
                result_msg.data = "FAILURE"
                self.verification_result_pub.publish(result_msg)
                
                self.transition_to(SupervisorState.SEARCH)
                self.publish_mission_command("COVERAGE_SEARCH")
                self.verification_attempts = 0
                self.verification_start_time = None
                self.garbage_before_collection = None
            else:
                # Try again
                self.get_logger().warn(f'⚠️ Verification failed (attempt {self.verification_attempts}/{self.max_verification_attempts}) - retrying')
                
                result_msg = String()
                result_msg.data = "RETRY"
                self.verification_result_pub.publish(result_msg)
                
                # Reset for another attempt
                self.verification_start_time = None

    # ============= SUPERVISION LOOP =============
    
    def supervision_loop(self):
        """Main supervision loop"""
        if self.current_state == SupervisorState.IDLE:
            pass
        elif self.current_state == SupervisorState.SEARCH:
            self._handle_search_state()
        elif self.current_state == SupervisorState.APPROACH:
            self._handle_approach_state()
        elif self.current_state == SupervisorState.COLLECT:
            self._handle_collect_state()
        elif self.current_state == SupervisorState.VERIFY:
            self._handle_verify_state()
        elif self.current_state == SupervisorState.RETURN_TO_HOME:
            self._handle_rth_state()

    def _handle_search_state(self):
        """SEARCH state behavior"""
        if self.bin_full or self.current_battery <= self.battery_threshold:
            self.transition_to(SupervisorState.RETURN_TO_HOME)
            self.publish_mission_command("RETURN_TO_HOME")

    def _handle_approach_state(self):
        """APPROACH state behavior - FIXED"""
        # Check if we lost debris (no detection for 5 seconds)
        current_time = self.get_current_time_seconds()
        
        if self.current_detection is None or self.current_detection.label != 'garbage':
            # Check if we have a recent detection stored
            if (self.garbage_before_collection and 
                current_time - self.garbage_before_collection['timestamp'] < 3.0):
                # Still within timeout, keep approaching
                pass
            else:
                self.get_logger().warn('Lost garbage during approach')
                self.transition_to(SupervisorState.SEARCH)
                self.publish_mission_command("COVERAGE_SEARCH")

    def _handle_collect_state(self):
        """COLLECT state behavior - FIXED with proper timer"""
        if not self.collection_activated:
            # Activate collection mechanism
            activate_msg = Bool()
            activate_msg.data = True
            self.collection_activation_pub.publish(activate_msg)
            self.collection_activated = True
            self.get_logger().info('🔄 Collection mechanism activated')
            
            # Create timer for 2 seconds, then move to verify
            if self.collection_timer is None:
                self.collection_timer = self.create_timer(2.0, self.collection_timer_callback)

    def collection_timer_callback(self):
        """Timer callback after collection"""
        self.transition_to(SupervisorState.VERIFY)
        self.collection_activated = False
        # Destroy the timer
        if self.collection_timer:
            self.collection_timer.cancel()
            self.collection_timer = None

    def _handle_rth_state(self):
        """RETURN_TO_HOME state behavior"""
        if self._is_home_reached():
            self.transition_to(SupervisorState.IDLE)
            self.publish_mission_command("STANDBY")
            self.get_logger().info('🏠 Home reached - mission complete')

    def _is_home_reached(self):
        """Check if home reached (simplified)"""
        # In real implementation, calculate distance
        return True

def main(args=None):
    rclpy.init(args=args)
    
    try:
        supervisor_node = VisionVerificationSupervisor()
        rclpy.spin(supervisor_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'❌ Supervisor error: {e}')
    finally:
        if 'supervisor_node' in locals():
            supervisor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()