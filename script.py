# First, let's create the main ROV system architecture code
import pandas as pd
import numpy as np

# Create the main system architecture code
main_system_code = """
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import threading
import time
import queue
import serial
import json
from scipy.spatial.transform import Rotation as R
from typing import Tuple, List, Dict, Optional

class ROVVisionSystem:
    def __init__(self, model_path: str = "yolov8n.pt", servo_port: str = "/dev/ttyUSB0"):
        # Initialize YOLOv8 model
        self.model = YOLO(model_path)
        
        # Initialize RealSense camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # Initialize servo communication
        self.servo_serial = serial.Serial(servo_port, 9600, timeout=1)
        
        # Threading and queues
        self.frame_queue = queue.Queue(maxsize=5)
        self.detection_queue = queue.Queue(maxsize=5)
        self.servo_command_queue = queue.Queue(maxsize=10)
        
        # System state
        self.running = False
        self.target_object = None
        self.camera_intrinsics = None
        
        # Control parameters
        self.target_distance = 0.5  # meters
        self.grab_threshold = 0.1   # meters
        self.servo_limits = {'pan': (-90, 90), 'tilt': (-45, 45), 'gripper': (0, 180)}
        
    def start_system(self):
        \"\"\"Start the ROV vision system\"\"\"
        self.running = True
        self.pipeline.start(self.config)
        
        # Get camera intrinsics
        profile = self.pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        self.camera_intrinsics = color_profile.get_intrinsics()
        
        # Start threads
        self.capture_thread = threading.Thread(target=self._capture_frames)
        self.detection_thread = threading.Thread(target=self._detect_objects)
        self.control_thread = threading.Thread(target=self._control_servos)
        
        self.capture_thread.start()
        self.detection_thread.start()
        self.control_thread.start()
        
        print("ROV Vision System Started")
        
    def stop_system(self):
        \"\"\"Stop the ROV vision system\"\"\"
        self.running = False
        self.pipeline.stop()
        self.servo_serial.close()
        
        # Join threads
        self.capture_thread.join()
        self.detection_thread.join()
        self.control_thread.join()
        
        print("ROV Vision System Stopped")
        
    def _capture_frames(self):
        \"\"\"Capture frames from RealSense camera\"\"\"
        while self.running:
            try:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    continue
                
                # Convert to numpy arrays
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                
                # Put in queue for processing
                if not self.frame_queue.full():
                    self.frame_queue.put({
                        'color': color_image,
                        'depth': depth_image,
                        'timestamp': time.time()
                    })
                    
            except Exception as e:
                print(f"Frame capture error: {e}")
                time.sleep(0.1)
    
    def _detect_objects(self):
        \"\"\"Detect objects using YOLOv8\"\"\"
        while self.running:
            try:
                if not self.frame_queue.empty():
                    frame_data = self.frame_queue.get()
                    color_image = frame_data['color']
                    depth_image = frame_data['depth']
                    
                    # Run YOLOv8 detection
                    results = self.model(color_image)
                    
                    # Process detections
                    detections = self._process_detections(results[0], depth_image)
                    
                    # Put detections in queue
                    if not self.detection_queue.full():
                        self.detection_queue.put({
                            'detections': detections,
                            'color_image': color_image,
                            'depth_image': depth_image,
                            'timestamp': frame_data['timestamp']
                        })
                        
            except Exception as e:
                print(f"Detection error: {e}")
                time.sleep(0.1)
    
    def _process_detections(self, results, depth_image) -> List[Dict]:
        \"\"\"Process YOLOv8 detection results\"\"\"
        detections = []
        
        if results.boxes is not None:
            for box in results.boxes:
                # Get bounding box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = box.conf[0].cpu().numpy()
                class_id = int(box.cls[0].cpu().numpy())
                class_name = self.model.names[class_id]
                
                # Calculate center point
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                # Get depth at center point
                depth_value = depth_image[center_y, center_x]
                distance = depth_value * 0.001  # Convert to meters
                
                # Calculate 3D position
                position_3d = self._pixel_to_3d(center_x, center_y, distance)
                
                detections.append({
                    'class_name': class_name,
                    'confidence': confidence,
                    'bbox': [x1, y1, x2, y2],
                    'center': [center_x, center_y],
                    'distance': distance,
                    'position_3d': position_3d
                })
        
        return detections
    
    def _pixel_to_3d(self, x: int, y: int, depth: float) -> Tuple[float, float, float]:
        \"\"\"Convert pixel coordinates to 3D world coordinates\"\"\"
        if self.camera_intrinsics is None:
            return (0, 0, 0)
        
        # Convert pixel to 3D point
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.camera_intrinsics, [x, y], depth
        )
        
        return tuple(point_3d)
    
    def _control_servos(self):
        \"\"\"Control servo motors based on object detection\"\"\"
        while self.running:
            try:
                if not self.detection_queue.empty():
                    detection_data = self.detection_queue.get()
                    detections = detection_data['detections']
                    
                    # Find target object (highest confidence detection)
                    if detections:
                        target = max(detections, key=lambda x: x['confidence'])
                        
                        # Calculate servo commands
                        servo_commands = self._calculate_servo_commands(target)
                        
                        # Send commands to servos
                        self._send_servo_commands(servo_commands)
                        
                        # Check if object is close enough to grab
                        if target['distance'] < self.grab_threshold:
                            self._grab_object()
                            
            except Exception as e:
                print(f"Servo control error: {e}")
                time.sleep(0.1)
    
    def _calculate_servo_commands(self, target: Dict) -> Dict:
        \"\"\"Calculate servo commands to track target\"\"\"
        center_x, center_y = target['center']
        image_center_x, image_center_y = 320, 240  # Assuming 640x480 resolution
        
        # Calculate error from center
        error_x = center_x - image_center_x
        error_y = center_y - image_center_y
        
        # Convert to servo angles (proportional control)
        pan_angle = -error_x * 0.1  # Adjust gain as needed
        tilt_angle = error_y * 0.1
        
        # Clamp to servo limits
        pan_angle = np.clip(pan_angle, *self.servo_limits['pan'])
        tilt_angle = np.clip(tilt_angle, *self.servo_limits['tilt'])
        
        return {
            'pan': pan_angle,
            'tilt': tilt_angle,
            'gripper': 0  # Keep gripper closed initially
        }
    
    def _send_servo_commands(self, commands: Dict):
        \"\"\"Send servo commands via serial\"\"\"
        try:
            command_json = json.dumps(commands)
            self.servo_serial.write(f"{command_json}\\n".encode())
        except Exception as e:
            print(f"Servo communication error: {e}")
    
    def _grab_object(self):
        \"\"\"Execute object grabbing sequence\"\"\"
        # Open gripper
        self._send_servo_commands({'gripper': 90})
        time.sleep(1)
        
        # Close gripper
        self._send_servo_commands({'gripper': 0})
        time.sleep(1)
        
        # Move to dustbin position (example coordinates)
        self._send_servo_commands({'pan': 0, 'tilt': 0})
        time.sleep(2)
        
        # Open gripper to drop object
        self._send_servo_commands({'gripper': 90})
        time.sleep(1)
        
        print("Object grabbed and placed in dustbin")


class ServoController:
    \"\"\"Arduino servo controller code (to be uploaded to Arduino)\"\"\"
    
    @staticmethod
    def get_arduino_code():
        return '''
#include <Servo.h>
#include <ArduinoJson.h>

Servo panServo;
Servo tiltServo;
Servo gripperServo;

const int PAN_PIN = 9;
const int TILT_PIN = 10;
const int GRIPPER_PIN = 11;

int panPos = 90;    // Center position
int tiltPos = 90;   // Center position
int gripperPos = 0; // Closed position

void setup() {
    Serial.begin(9600);
    
    panServo.attach(PAN_PIN);
    tiltServo.attach(TILT_PIN);
    gripperServo.attach(GRIPPER_PIN);
    
    // Initialize to center positions
    panServo.write(panPos);
    tiltServo.write(tiltPos);
    gripperServo.write(gripperPos);
    
    Serial.println("ROV Servo Controller Ready");
}

void loop() {
    if (Serial.available() > 0) {
        String jsonString = Serial.readStringUntil('\\n');
        
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, jsonString);
        
        if (error) {
            Serial.print("JSON parse error: ");
            Serial.println(error.c_str());
            return;
        }
        
        // Update servo positions
        if (doc.containsKey("pan")) {
            panPos = constrain(doc["pan"], 0, 180);
            panServo.write(panPos);
        }
        
        if (doc.containsKey("tilt")) {
            tiltPos = constrain(doc["tilt"], 0, 180);
            tiltServo.write(tiltPos);
        }
        
        if (doc.containsKey("gripper")) {
            gripperPos = constrain(doc["gripper"], 0, 180);
            gripperServo.write(gripperPos);
        }
        
        delay(20); // Small delay for servo movement
    }
}
'''


def main():
    # Initialize and start the ROV system
    rov_system = ROVVisionSystem()
    
    try:
        rov_system.start_system()
        
        # Keep system running
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Stopping ROV system...")
        rov_system.stop_system()


if __name__ == "__main__":
    main()
"""

# Save the main system code to a file
with open('rov_vision_system.py', 'w') as f:
    f.write(main_system_code)

print("Main ROV system code created successfully!")
print("File saved as: rov_vision_system.py")
