#!/usr/bin/env python3
"""
FIXED PERCEPTION NODE - HANDLES DYNAMIC INPUT DIMENSIONS
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from pathlib import Path
import os

# ROS 2 messages
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

# Your custom message
from usv_autonomy.msg import Debris

try:
    import onnxruntime as ort
    ONNX_AVAILABLE = True
except ImportError:
    ONNX_AVAILABLE = False
    print("ONNX Runtime not available - install with: pip install onnxruntime")

class FixedPerceptionNode(Node):
    """
    Fixed perception node that handles dynamic input dimensions
    """
    
    def __init__(self):
        super().__init__('perception_node')
        
        # Parameters with proper default values
        self.declare_parameter('model_path', '/home/ama/usv_ws/models/water_surface_detection.onnx')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/detections/debris')
        self.declare_parameter('img_size', 640)  # Default YOLOv8 size
        
        # Load parameters
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.img_size = self.get_parameter('img_size').value  # Use parameter value
        
        # Check if ONNX Runtime is available
        if not ONNX_AVAILABLE:
            self.get_logger().error('❌ ONNX Runtime not installed!')
            self.get_logger().info('💡 Install with: pip install onnxruntime')
            return
        
        # Check if model file exists
        if not os.path.exists(model_path):
            self.get_logger().error(f'❌ Model file not found: {model_path}')
            self.get_logger().info('💡 Please download your ONNX model to this location')
            return
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Load ONNX model
        self.get_logger().info(f'Loading ONNX model from: {model_path}')
        try:
            self.session = ort.InferenceSession(model_path)
            self.get_logger().info('✅ ONNX model loaded successfully')
            
            # Get model info
            self.input_name = self.session.get_inputs()[0].name
            input_shape = self.session.get_inputs()[0].shape
            self.get_logger().info(f'Model input shape: {input_shape}')
            
            # Handle dynamic dimensions - use parameter value or try to extract from shape
            if len(input_shape) == 4:
                # Try to get size from shape [batch, channels, height, width]
                if isinstance(input_shape[2], int) and input_shape[2] > 0:
                    self.img_size = input_shape[2]
                elif isinstance(input_shape[3], int) and input_shape[3] > 0:
                    self.img_size = input_shape[3]
            
            self.get_logger().info(f'Using image size: {self.img_size}x{self.img_size}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load ONNX model: {e}')
            return
        
        # Your class mapping
        self.class_names = {
            0: 'aquatic_animal',
            1: 'garbage', 
            2: 'plants'
        }
        
        # Publishers and Subscribers
        self.detection_pub = self.create_publisher(Debris, output_topic, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        
        # Counter for debugging
        self.frame_count = 0
        
        self.get_logger().info('🎯 Perception Node initialized successfully!')
        self.get_logger().info(f'📁 Using model: {Path(model_path).name}')
        self.get_logger().info(f'🎯 Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'📐 Image size: {self.img_size}x{self.img_size}')

    def preprocess_image(self, cv_image):
        """Preprocess image for ONNX model - FIXED"""
        # Ensure img_size is an integer
        if not isinstance(self.img_size, int):
            self.img_size = 640  # Default fallback
            
        # FIXED: Use integer tuple for resize dimensions
        target_size = (self.img_size, self.img_size)
        img_resized = cv2.resize(cv_image, target_size)
        
        # Normalize (0-1 range)
        img_normalized = img_resized.astype(np.float32) / 255.0
        
        # Convert to CHW format
        img_chw = np.transpose(img_normalized, (2, 0, 1))
        
        # Add batch dimension
        img_batch = np.expand_dims(img_chw, axis=0)
        
        return img_batch

    def postprocess_yolov8_output(self, outputs, original_shape):
        """
        Postprocess YOLOv8 ONNX output
        Handle different output formats
        """
        detections = []
        
        try:
            # Get the first output
            predictions = outputs[0]
            
            # Handle different output shapes
            if len(predictions.shape) == 3:
                # Standard YOLOv8 output: [1, 7, 8400] or similar
                predictions = np.squeeze(predictions)  # Remove batch dimension
                
                if predictions.shape[0] == 7:  # Your 3-class model
                    # [x_center, y_center, width, height, confidence, class0, class1, class2]
                    confidence_scores = predictions[4:5, :].max(axis=0)
                    keep = confidence_scores > self.confidence_threshold
                    
                    if not np.any(keep):
                        return detections
                    
                    filtered_preds = predictions[:, keep]
                    filtered_confidences = confidence_scores[keep]
                    
                    # Get class IDs
                    class_scores = predictions[5:, :]
                    class_ids = class_scores[:, keep].argmax(axis=0)
                    
                    # Convert to detections
                    for i in range(filtered_preds.shape[1]):
                        x_center, y_center, width, height = filtered_preds[0:4, i]
                        confidence = filtered_confidences[i]
                        class_id = class_ids[i]
                        
                        # Convert coordinates
                        orig_h, orig_w = original_shape
                        scale_x = orig_w / self.img_size
                        scale_y = orig_h / self.img_size
                        
                        bbox_center_x = x_center * scale_x
                        bbox_center_y = y_center * scale_y
                        bbox_width = width * scale_x
                        bbox_height = height * scale_y
                        bbox_area = bbox_width * bbox_height
                        
                        detections.append({
                            'class_id': int(class_id),
                            'confidence': float(confidence),
                            'bbox_center_x': float(bbox_center_x),
                            'bbox_center_y': float(bbox_center_y),
                            'bbox_area': float(bbox_area)
                        })
            
            elif len(predictions.shape) == 2:
                # Alternative format: [num_detections, 6] where 6 = [x1, y1, x2, y2, confidence, class]
                for detection in predictions:
                    if detection[4] > self.confidence_threshold:  # confidence
                        x1, y1, x2, y2, confidence, class_id = detection
                        
                        # Convert to center format
                        bbox_center_x = (x1 + x2) / 2
                        bbox_center_y = (y1 + y2) / 2
                        bbox_width = x2 - x1
                        bbox_height = y2 - y1
                        bbox_area = bbox_width * bbox_height
                        
                        # Scale to original image size
                        orig_h, orig_w = original_shape
                        scale_x = orig_w / self.img_size
                        scale_y = orig_h / self.img_size
                        
                        detections.append({
                            'class_id': int(class_id),
                            'confidence': float(confidence),
                            'bbox_center_x': float(bbox_center_x * scale_x),
                            'bbox_center_y': float(bbox_center_y * scale_y),
                            'bbox_area': float(bbox_area * scale_x * scale_y)
                        })
                        
        except Exception as e:
            self.get_logger().error(f'Postprocessing error: {e}')
            
        return detections

    def debug_model_output(self, outputs):
        """Debug function to understand model output format"""
        self.get_logger().info("=== MODEL OUTPUT DEBUG ===")
        for i, output in enumerate(outputs):
            self.get_logger().info(f"Output {i}: shape={output.shape}, dtype={output.dtype}")
            self.get_logger().info(f"Value range: {output.min():.3f} to {output.max():.3f}")
        
        # Only log this once
        if not hasattr(self, 'debug_done'):
            self.debug_done = True

    def image_callback(self, msg):
        """Process image with ONNX model"""
        self.frame_count += 1
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            original_shape = cv_image.shape[:2]  # (height, width)
            
            # Debug first few frames
            if self.frame_count <= 3:
                self.get_logger().info(f'Frame {self.frame_count}: {original_shape} -> {self.img_size}x{self.img_size}')
            
            # Preprocess
            input_tensor = self.preprocess_image(cv_image)
            
            # Run inference
            outputs = self.session.run(None, {self.input_name: input_tensor})
            
            # Debug output on first frame
            if self.frame_count == 1:
                self.debug_model_output(outputs)
            
            # Postprocess
            detections = self.postprocess_yolov8_output(outputs, original_shape)
            
            # Publish detections
            for detection in detections:
                debris_msg = Debris()
                debris_msg.header = msg.header
                debris_msg.label = self.class_names.get(detection['class_id'], 'unknown')
                debris_msg.confidence = detection['confidence']
                debris_msg.bbox_center_x = detection['bbox_center_x']
                debris_msg.bbox_center_y = detection['bbox_center_y']
                debris_msg.bbox_area = detection['bbox_area']
                
                self.detection_pub.publish(debris_msg)
                
                self.get_logger().info(
                    f'🎯 Detected: {debris_msg.label} '
                    f'at ({debris_msg.bbox_center_x:.1f}, {debris_msg.bbox_center_y:.1f}) '
                    f'conf: {debris_msg.confidence:.2f}'
                )
                
            if len(detections) > 0:
                self.get_logger().info(f'📦 Found {len(detections)} objects')
            elif self.frame_count % 30 == 0:  # Log every 30 frames if no detections
                self.get_logger().info('🔍 Scanning for debris...')
                
        except Exception as e:
            self.get_logger().error(f'❌ Image processing error: {e}')
            # Log more details for debugging
            import traceback
            self.get_logger().error(f'Stack trace: {traceback.format_exc()}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FixedPerceptionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'❌ Perception node error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()#!/usr/bin/env python3
"""
FIXED PERCEPTION NODE - HANDLES DYNAMIC INPUT DIMENSIONS
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from pathlib import Path
import os

# ROS 2 messages
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

# Your custom message
from usv_autonomy.msg import Debris

try:
    import onnxruntime as ort
    ONNX_AVAILABLE = True
except ImportError:
    ONNX_AVAILABLE = False
    print("ONNX Runtime not available - install with: pip install onnxruntime")

class FixedPerceptionNode(Node):
    """
    Fixed perception node that handles dynamic input dimensions
    """
    
    def __init__(self):
        super().__init__('perception_node')
        
        # Parameters with proper default values
        self.declare_parameter('model_path', '/home/ama/usv_ws/models/water_surface_detection.onnx')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/detections/debris')
        self.declare_parameter('img_size', 640)  # Default YOLOv8 size
        
        # Load parameters
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.img_size = self.get_parameter('img_size').value  # Use parameter value
        
        # Check if ONNX Runtime is available
        if not ONNX_AVAILABLE:
            self.get_logger().error('❌ ONNX Runtime not installed!')
            self.get_logger().info('💡 Install with: pip install onnxruntime')
            return
        
        # Check if model file exists
        if not os.path.exists(model_path):
            self.get_logger().error(f'❌ Model file not found: {model_path}')
            self.get_logger().info('💡 Please download your ONNX model to this location')
            return
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Load ONNX model
        self.get_logger().info(f'Loading ONNX model from: {model_path}')
        try:
            self.session = ort.InferenceSession(model_path)
            self.get_logger().info('✅ ONNX model loaded successfully')
            
            # Get model info
            self.input_name = self.session.get_inputs()[0].name
            input_shape = self.session.get_inputs()[0].shape
            self.get_logger().info(f'Model input shape: {input_shape}')
            
            # Handle dynamic dimensions - use parameter value or try to extract from shape
            if len(input_shape) == 4:
                # Try to get size from shape [batch, channels, height, width]
                if isinstance(input_shape[2], int) and input_shape[2] > 0:
                    self.img_size = input_shape[2]
                elif isinstance(input_shape[3], int) and input_shape[3] > 0:
                    self.img_size = input_shape[3]
            
            self.get_logger().info(f'Using image size: {self.img_size}x{self.img_size}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load ONNX model: {e}')
            return
        
        # Your class mapping
        self.class_names = {
            0: 'aquatic_animal',
            1: 'garbage', 
            2: 'plants'
        }
        
        # Publishers and Subscribers
        self.detection_pub = self.create_publisher(Debris, output_topic, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        
        # Counter for debugging
        self.frame_count = 0
        
        self.get_logger().info('🎯 Perception Node initialized successfully!')
        self.get_logger().info(f'📁 Using model: {Path(model_path).name}')
        self.get_logger().info(f'🎯 Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'📐 Image size: {self.img_size}x{self.img_size}')

    def preprocess_image(self, cv_image):
        """Preprocess image for ONNX model - FIXED"""
        # Ensure img_size is an integer
        if not isinstance(self.img_size, int):
            self.img_size = 640  # Default fallback
            
        # FIXED: Use integer tuple for resize dimensions
        target_size = (self.img_size, self.img_size)
        img_resized = cv2.resize(cv_image, target_size)
        
        # Normalize (0-1 range)
        img_normalized = img_resized.astype(np.float32) / 255.0
        
        # Convert to CHW format
        img_chw = np.transpose(img_normalized, (2, 0, 1))
        
        # Add batch dimension
        img_batch = np.expand_dims(img_chw, axis=0)
        
        return img_batch

    def postprocess_yolov8_output(self, outputs, original_shape):
        """
        Postprocess YOLOv8 ONNX output
        Handle different output formats
        """
        detections = []
        
        try:
            # Get the first output
            predictions = outputs[0]
            
            # Handle different output shapes
            if len(predictions.shape) == 3:
                # Standard YOLOv8 output: [1, 7, 8400] or similar
                predictions = np.squeeze(predictions)  # Remove batch dimension
                
                if predictions.shape[0] == 7:  # Your 3-class model
                    # [x_center, y_center, width, height, confidence, class0, class1, class2]
                    confidence_scores = predictions[4:5, :].max(axis=0)
                    keep = confidence_scores > self.confidence_threshold
                    
                    if not np.any(keep):
                        return detections
                    
                    filtered_preds = predictions[:, keep]
                    filtered_confidences = confidence_scores[keep]
                    
                    # Get class IDs
                    class_scores = predictions[5:, :]
                    class_ids = class_scores[:, keep].argmax(axis=0)
                    
                    # Convert to detections
                    for i in range(filtered_preds.shape[1]):
                        x_center, y_center, width, height = filtered_preds[0:4, i]
                        confidence = filtered_confidences[i]
                        class_id = class_ids[i]
                        
                        # Convert coordinates
                        orig_h, orig_w = original_shape
                        scale_x = orig_w / self.img_size
                        scale_y = orig_h / self.img_size
                        
                        bbox_center_x = x_center * scale_x
                        bbox_center_y = y_center * scale_y
                        bbox_width = width * scale_x
                        bbox_height = height * scale_y
                        bbox_area = bbox_width * bbox_height
                        
                        detections.append({
                            'class_id': int(class_id),
                            'confidence': float(confidence),
                            'bbox_center_x': float(bbox_center_x),
                            'bbox_center_y': float(bbox_center_y),
                            'bbox_area': float(bbox_area)
                        })
            
            elif len(predictions.shape) == 2:
                # Alternative format: [num_detections, 6] where 6 = [x1, y1, x2, y2, confidence, class]
                for detection in predictions:
                    if detection[4] > self.confidence_threshold:  # confidence
                        x1, y1, x2, y2, confidence, class_id = detection
                        
                        # Convert to center format
                        bbox_center_x = (x1 + x2) / 2
                        bbox_center_y = (y1 + y2) / 2
                        bbox_width = x2 - x1
                        bbox_height = y2 - y1
                        bbox_area = bbox_width * bbox_height
                        
                        # Scale to original image size
                        orig_h, orig_w = original_shape
                        scale_x = orig_w / self.img_size
                        scale_y = orig_h / self.img_size
                        
                        detections.append({
                            'class_id': int(class_id),
                            'confidence': float(confidence),
                            'bbox_center_x': float(bbox_center_x * scale_x),
                            'bbox_center_y': float(bbox_center_y * scale_y),
                            'bbox_area': float(bbox_area * scale_x * scale_y)
                        })
                        
        except Exception as e:
            self.get_logger().error(f'Postprocessing error: {e}')
            
        return detections

    def debug_model_output(self, outputs):
        """Debug function to understand model output format"""
        self.get_logger().info("=== MODEL OUTPUT DEBUG ===")
        for i, output in enumerate(outputs):
            self.get_logger().info(f"Output {i}: shape={output.shape}, dtype={output.dtype}")
            self.get_logger().info(f"Value range: {output.min():.3f} to {output.max():.3f}")
        
        # Only log this once
        if not hasattr(self, 'debug_done'):
            self.debug_done = True

    def image_callback(self, msg):
        """Process image with ONNX model"""
        self.frame_count += 1
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            original_shape = cv_image.shape[:2]  # (height, width)
            
            # Debug first few frames
            if self.frame_count <= 3:
                self.get_logger().info(f'Frame {self.frame_count}: {original_shape} -> {self.img_size}x{self.img_size}')
            
            # Preprocess
            input_tensor = self.preprocess_image(cv_image)
            
            # Run inference
            outputs = self.session.run(None, {self.input_name: input_tensor})
            
            # Debug output on first frame
            if self.frame_count == 1:
                self.debug_model_output(outputs)
            
            # Postprocess
            detections = self.postprocess_yolov8_output(outputs, original_shape)
            
            # Publish detections
            for detection in detections:
                debris_msg = Debris()
                debris_msg.header = msg.header
                debris_msg.label = self.class_names.get(detection['class_id'], 'unknown')
                debris_msg.confidence = detection['confidence']
                debris_msg.bbox_center_x = detection['bbox_center_x']
                debris_msg.bbox_center_y = detection['bbox_center_y']
                debris_msg.bbox_area = detection['bbox_area']
                
                self.detection_pub.publish(debris_msg)
                
                self.get_logger().info(
                    f'🎯 Detected: {debris_msg.label} '
                    f'at ({debris_msg.bbox_center_x:.1f}, {debris_msg.bbox_center_y:.1f}) '
                    f'conf: {debris_msg.confidence:.2f}'
                )
                
            if len(detections) > 0:
                self.get_logger().info(f'📦 Found {len(detections)} objects')
            elif self.frame_count % 30 == 0:  # Log every 30 frames if no detections
                self.get_logger().info('🔍 Scanning for debris...')
                
        except Exception as e:
            self.get_logger().error(f'❌ Image processing error: {e}')
            # Log more details for debugging
            import traceback
            self.get_logger().error(f'Stack trace: {traceback.format_exc()}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FixedPerceptionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'❌ Perception node error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()#!/usr/bin/env python3
"""
FIXED PERCEPTION NODE - HANDLES DYNAMIC INPUT DIMENSIONS
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from pathlib import Path
import os

# ROS 2 messages
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

# Your custom message
from usv_autonomy.msg import Debris

try:
    import onnxruntime as ort
    ONNX_AVAILABLE = True
except ImportError:
    ONNX_AVAILABLE = False
    print("ONNX Runtime not available - install with: pip install onnxruntime")

class FixedPerceptionNode(Node):
    """
    Fixed perception node that handles dynamic input dimensions
    """
    
    def __init__(self):
        super().__init__('perception_node')
        
        # Parameters with proper default values
        self.declare_parameter('model_path', '/home/ama/usv_ws/models/water_surface_detection.onnx')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/detections/debris')
        self.declare_parameter('img_size', 640)  # Default YOLOv8 size
        
        # Load parameters
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.img_size = self.get_parameter('img_size').value  # Use parameter value
        
        # Check if ONNX Runtime is available
        if not ONNX_AVAILABLE:
            self.get_logger().error('❌ ONNX Runtime not installed!')
            self.get_logger().info('💡 Install with: pip install onnxruntime')
            return
        
        # Check if model file exists
        if not os.path.exists(model_path):
            self.get_logger().error(f'❌ Model file not found: {model_path}')
            self.get_logger().info('💡 Please download your ONNX model to this location')
            return
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Load ONNX model
        self.get_logger().info(f'Loading ONNX model from: {model_path}')
        try:
            self.session = ort.InferenceSession(model_path)
            self.get_logger().info('✅ ONNX model loaded successfully')
            
            # Get model info
            self.input_name = self.session.get_inputs()[0].name
            input_shape = self.session.get_inputs()[0].shape
            self.get_logger().info(f'Model input shape: {input_shape}')
            
            # Handle dynamic dimensions - use parameter value or try to extract from shape
            if len(input_shape) == 4:
                # Try to get size from shape [batch, channels, height, width]
                if isinstance(input_shape[2], int) and input_shape[2] > 0:
                    self.img_size = input_shape[2]
                elif isinstance(input_shape[3], int) and input_shape[3] > 0:
                    self.img_size = input_shape[3]
            
            self.get_logger().info(f'Using image size: {self.img_size}x{self.img_size}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load ONNX model: {e}')
            return
        
        # Your class mapping
        self.class_names = {
            0: 'aquatic_animal',
            1: 'garbage', 
            2: 'plants'
        }
        
        # Publishers and Subscribers
        self.detection_pub = self.create_publisher(Debris, output_topic, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        
        # Counter for debugging
        self.frame_count = 0
        
        self.get_logger().info('🎯 Perception Node initialized successfully!')
        self.get_logger().info(f'📁 Using model: {Path(model_path).name}')
        self.get_logger().info(f'🎯 Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'📐 Image size: {self.img_size}x{self.img_size}')

    def preprocess_image(self, cv_image):
        """Preprocess image for ONNX model - FIXED"""
        # Ensure img_size is an integer
        if not isinstance(self.img_size, int):
            self.img_size = 640  # Default fallback
            
        # FIXED: Use integer tuple for resize dimensions
        target_size = (self.img_size, self.img_size)
        img_resized = cv2.resize(cv_image, target_size)
        
        # Normalize (0-1 range)
        img_normalized = img_resized.astype(np.float32) / 255.0
        
        # Convert to CHW format
        img_chw = np.transpose(img_normalized, (2, 0, 1))
        
        # Add batch dimension
        img_batch = np.expand_dims(img_chw, axis=0)
        
        return img_batch

    def postprocess_yolov8_output(self, outputs, original_shape):
        """
        Postprocess YOLOv8 ONNX output
        Handle different output formats
        """
        detections = []
        
        try:
            # Get the first output
            predictions = outputs[0]
            
            # Handle different output shapes
            if len(predictions.shape) == 3:
                # Standard YOLOv8 output: [1, 7, 8400] or similar
                predictions = np.squeeze(predictions)  # Remove batch dimension
                
                if predictions.shape[0] == 7:  # Your 3-class model
                    # [x_center, y_center, width, height, confidence, class0, class1, class2]
                    confidence_scores = predictions[4:5, :].max(axis=0)
                    keep = confidence_scores > self.confidence_threshold
                    
                    if not np.any(keep):
                        return detections
                    
                    filtered_preds = predictions[:, keep]
                    filtered_confidences = confidence_scores[keep]
                    
                    # Get class IDs
                    class_scores = predictions[5:, :]
                    class_ids = class_scores[:, keep].argmax(axis=0)
                    
                    # Convert to detections
                    for i in range(filtered_preds.shape[1]):
                        x_center, y_center, width, height = filtered_preds[0:4, i]
                        confidence = filtered_confidences[i]
                        class_id = class_ids[i]
                        
                        # Convert coordinates
                        orig_h, orig_w = original_shape
                        scale_x = orig_w / self.img_size
                        scale_y = orig_h / self.img_size
                        
                        bbox_center_x = x_center * scale_x
                        bbox_center_y = y_center * scale_y
                        bbox_width = width * scale_x
                        bbox_height = height * scale_y
                        bbox_area = bbox_width * bbox_height
                        
                        detections.append({
                            'class_id': int(class_id),
                            'confidence': float(confidence),
                            'bbox_center_x': float(bbox_center_x),
                            'bbox_center_y': float(bbox_center_y),
                            'bbox_area': float(bbox_area)
                        })
            
            elif len(predictions.shape) == 2:
                # Alternative format: [num_detections, 6] where 6 = [x1, y1, x2, y2, confidence, class]
                for detection in predictions:
                    if detection[4] > self.confidence_threshold:  # confidence
                        x1, y1, x2, y2, confidence, class_id = detection
                        
                        # Convert to center format
                        bbox_center_x = (x1 + x2) / 2
                        bbox_center_y = (y1 + y2) / 2
                        bbox_width = x2 - x1
                        bbox_height = y2 - y1
                        bbox_area = bbox_width * bbox_height
                        
                        # Scale to original image size
                        orig_h, orig_w = original_shape
                        scale_x = orig_w / self.img_size
                        scale_y = orig_h / self.img_size
                        
                        detections.append({
                            'class_id': int(class_id),
                            'confidence': float(confidence),
                            'bbox_center_x': float(bbox_center_x * scale_x),
                            'bbox_center_y': float(bbox_center_y * scale_y),
                            'bbox_area': float(bbox_area * scale_x * scale_y)
                        })
                        
        except Exception as e:
            self.get_logger().error(f'Postprocessing error: {e}')
            
        return detections

    def debug_model_output(self, outputs):
        """Debug function to understand model output format"""
        self.get_logger().info("=== MODEL OUTPUT DEBUG ===")
        for i, output in enumerate(outputs):
            self.get_logger().info(f"Output {i}: shape={output.shape}, dtype={output.dtype}")
            self.get_logger().info(f"Value range: {output.min():.3f} to {output.max():.3f}")
        
        # Only log this once
        if not hasattr(self, 'debug_done'):
            self.debug_done = True

    def image_callback(self, msg):
        """Process image with ONNX model"""
        self.frame_count += 1
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            original_shape = cv_image.shape[:2]  # (height, width)
            
            # Debug first few frames
            if self.frame_count <= 3:
                self.get_logger().info(f'Frame {self.frame_count}: {original_shape} -> {self.img_size}x{self.img_size}')
            
            # Preprocess
            input_tensor = self.preprocess_image(cv_image)
            
            # Run inference
            outputs = self.session.run(None, {self.input_name: input_tensor})
            
            # Debug output on first frame
            if self.frame_count == 1:
                self.debug_model_output(outputs)
            
            # Postprocess
            detections = self.postprocess_yolov8_output(outputs, original_shape)
            
            # Publish detections
            for detection in detections:
                debris_msg = Debris()
                debris_msg.header = msg.header
                debris_msg.label = self.class_names.get(detection['class_id'], 'unknown')
                debris_msg.confidence = detection['confidence']
                debris_msg.bbox_center_x = detection['bbox_center_x']
                debris_msg.bbox_center_y = detection['bbox_center_y']
                debris_msg.bbox_area = detection['bbox_area']
                
                self.detection_pub.publish(debris_msg)
                
                self.get_logger().info(
                    f'🎯 Detected: {debris_msg.label} '
                    f'at ({debris_msg.bbox_center_x:.1f}, {debris_msg.bbox_center_y:.1f}) '
                    f'conf: {debris_msg.confidence:.2f}'
                )
                
            if len(detections) > 0:
                self.get_logger().info(f'📦 Found {len(detections)} objects')
            elif self.frame_count % 30 == 0:  # Log every 30 frames if no detections
                self.get_logger().info('🔍 Scanning for debris...')
                
        except Exception as e:
            self.get_logger().error(f'❌ Image processing error: {e}')
            # Log more details for debugging
            import traceback
            self.get_logger().error(f'Stack trace: {traceback.format_exc()}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FixedPerceptionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'❌ Perception node error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()