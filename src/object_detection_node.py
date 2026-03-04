#!/usr/bin/env python3
"""
Object Detection Node using YOLOv8
Works with both Gazebo camera and real hardware cameras
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import threading

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Declare parameters as strings (matching launch file types)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', '0.5')  # String, not float
        self.declare_parameter('enable_visualization', 'true')  # String, not bool
        
        # Get parameters and convert to proper types
        self.camera_topic = self.get_parameter('camera_topic').value
        model_path = self.get_parameter('model_path').value
        conf_threshold = self.get_parameter('confidence_threshold').value
        self.confidence_threshold = float(conf_threshold)
            
        enable_vis = self.get_parameter('enable_visualization').value
        self.enable_visualization = enable_vis.lower() == 'true'
        
        # Initialize YOLO
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"Loaded YOLO model: {model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            self.model = None
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 
            self.camera_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        self.detections_pub = self.create_publisher(String, '/detections', 10)
        self.processed_image_pub = self.create_publisher(Image, '/object_detection/image', 10)
        self.obstacles_pub = self.create_publisher(String, '/obstacles_detected', 10)
        
        self.get_logger().info(f"Object Detection Node initialized (Camera: {self.camera_topic})")
        
        # Thread-safe lock for inference
        self.inference_lock = threading.Lock()
        self.last_frame = None
        self.inference_running = False
        
    def image_callback(self, msg):
        """Process incoming image and detect objects"""
        try:
            if self.model is None:
                return
                
            # Skip frame if inference still running (prevent backlog)
            if self.inference_running:
                return
                
            # Convert ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_frame = frame.copy()
            
            # Run inference in thread to avoid blocking
            threading.Thread(target=self.run_inference, args=(frame, msg.header.stamp), daemon=True).start()
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def run_inference(self, frame, timestamp):
        """Run YOLO inference"""
        self.inference_running = True
        try:
            with self.inference_lock:
                # Get frame dimensions
                h, w = frame.shape[:2]
                
                # Run inference (reduced resolution for speed - 256 fastest, 416 balanced)
                results = self.model(frame, verbose=False, imgsz=256)
                
                # Process detections
                detections = []
                obstacles = []
                annotated_frame = frame.copy()
                
                if results and len(results) > 0:
                    result = results[0]
                    
                    if result.boxes is not None:
                        for box in result.boxes:
                            confidence = float(box.conf)
                            
                            if confidence > self.confidence_threshold:
                                # Get detection info
                                class_id = int(box.cls)
                                class_name = result.names[class_id]
                                coords = box.xyxy[0].cpu().numpy()
                                x1, y1, x2, y2 = coords.astype(int)
                                
                                # Calculate center and distance estimate
                                cx = (x1 + x2) // 2
                                cy = (y1 + y2) // 2
                                box_width = x2 - x1
                                
                                # Estimate relative position (normalized)
                                rel_x = (cx - w/2) / w
                                rel_y = (cy - h/2) / h
                                
                                detection_data = {
                                    'class': class_name,
                                    'confidence': float(confidence),
                                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                                    'center': [cx, cy],
                                    'relative_position': [rel_x, rel_y]
                                }
                                detections.append(detection_data)

                                # Check if obstacle (anything not far away or small)
                                if box_width > 50:  # Significant object in frame
                                    obstacles.append(class_name)
                                
                                # Draw bounding box if visualization enabled
                                if self.enable_visualization:
                                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                    label = f"{class_name} {confidence:.2f}"
                                    cv2.putText(annotated_frame, label, (x1, y1-10),
                                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Publish detections
                detections_msg = String()
                detections_msg.data = str(detections)
                self.detections_pub.publish(detections_msg)
                
                # Publish obstacles (always publish so sensor_fusion can reset)
                obstacles_msg = String()
                if obstacles:
                    detected_str = ', '.join(set(obstacles))
                    obstacles_msg.data = f"Obstacles detected: {detected_str}"
                    # Bold terminal print for visibility
                    self.get_logger().info(
                        f'\n========================================\n'
                        f'  OBJECT DETECTED: {detected_str}\n'
                        f'  Count: {len(obstacles)} object(s) in frame\n'
                        f'  Robot will STOP for 10 seconds!\n'
                        f'========================================')
                else:
                    obstacles_msg.data = "clear"
                self.obstacles_pub.publish(obstacles_msg)
                
                # Publish annotated image
                if self.enable_visualization:
                    annotated_img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                    annotated_img_msg.header.stamp = timestamp
                    self.processed_image_pub.publish(annotated_img_msg)
                
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
        finally:
            self.inference_running = False

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
