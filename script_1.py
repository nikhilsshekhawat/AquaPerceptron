# Create installation requirements and setup scripts
requirements_txt = """
# ROV Vision System Requirements
# Computer Vision and Deep Learning
ultralytics>=8.0.0
opencv-python>=4.5.0
numpy>=1.21.0
torch>=1.12.0
torchvision>=0.13.0

# Intel RealSense
pyrealsense2>=2.50.0

# Communication and Control
pyserial>=3.5
scipy>=1.7.0

# Utilities
pyyaml>=6.0
pillow>=8.0.0
matplotlib>=3.5.0
"""

# Create setup script
setup_script = """
#!/bin/bash

# ROV Vision System Setup Script
echo "Setting up ROV Vision System..."

# Update system packages
sudo apt-get update

# Install Python dependencies
echo "Installing Python dependencies..."
pip install -r requirements.txt

# Install Intel RealSense SDK
echo "Installing Intel RealSense SDK..."
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg

# Install Arduino IDE for servo controller programming
echo "Installing Arduino IDE..."
wget -O arduino.tar.xz https://downloads.arduino.cc/arduino-1.8.19-linux64.tar.xz
tar -xf arduino.tar.xz
sudo mv arduino-1.8.19 /opt/arduino
sudo /opt/arduino/install.sh

# Setup permissions for serial communication
sudo usermod -a -G dialout $USER

echo "Setup complete! Please reboot your system."
echo "After reboot, run: python rov_vision_system.py"
"""

# Create calibration script
calibration_script = """
import cv2
import numpy as np
import pyrealsense2 as rs
import json
import time

class ROVCalibration:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
    def calibrate_camera(self):
        \"\"\"Calibrate the RealSense camera\"\"\"
        print("Starting camera calibration...")
        
        # Start pipeline
        self.pipeline.start(self.config)
        
        # Get camera intrinsics
        profile = self.pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        intrinsics = color_profile.get_intrinsics()
        
        # Save calibration data
        calibration_data = {
            'width': intrinsics.width,
            'height': intrinsics.height,
            'fx': intrinsics.fx,
            'fy': intrinsics.fy,
            'ppx': intrinsics.ppx,
            'ppy': intrinsics.ppy,
            'model': str(intrinsics.model),
            'coeffs': intrinsics.coeffs
        }
        
        with open('camera_calibration.json', 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        print("Camera calibration saved to camera_calibration.json")
        
        # Stop pipeline
        self.pipeline.stop()
        
    def test_depth_accuracy(self):
        \"\"\"Test depth measurement accuracy\"\"\"
        print("Testing depth accuracy...")
        
        self.pipeline.start(self.config)
        
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    continue
                
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                
                # Get depth at center
                height, width = depth_image.shape
                center_depth = depth_image[height//2, width//2]
                distance_m = center_depth * 0.001
                
                # Display
                cv2.circle(color_image, (width//2, height//2), 5, (0, 255, 0), -1)
                cv2.putText(color_image, f"Distance: {distance_m:.2f}m", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                cv2.imshow('Depth Test', color_image)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()
            
    def calibrate_servo_range(self):
        \"\"\"Calibrate servo movement range\"\"\"
        print("Servo calibration instructions:")
        print("1. Connect Arduino with servo controller code")
        print("2. Position ROV gripper in center position")
        print("3. Note the pan/tilt angles for maximum range")
        print("4. Update servo_limits in the main code")
        
        # Interactive calibration would go here
        # For now, just provide guidance
        
        servo_config = {
            'pan': {'min': -90, 'max': 90, 'center': 0},
            'tilt': {'min': -45, 'max': 45, 'center': 0},
            'gripper': {'min': 0, 'max': 180, 'open': 90, 'closed': 0}
        }
        
        with open('servo_calibration.json', 'w') as f:
            json.dump(servo_config, f, indent=2)
        
        print("Servo calibration template saved to servo_calibration.json")

if __name__ == "__main__":
    calibrator = ROVCalibration()
    
    print("ROV Calibration System")
    print("1. Camera calibration")
    print("2. Depth accuracy test")
    print("3. Servo calibration")
    
    choice = input("Enter choice (1-3): ")
    
    if choice == '1':
        calibrator.calibrate_camera()
    elif choice == '2':
        calibrator.test_depth_accuracy()
    elif choice == '3':
        calibrator.calibrate_servo_range()
    else:
        print("Invalid choice")
"""

# Create configuration file
config_yaml = """
# ROV Vision System Configuration

# Camera Settings
camera:
  width: 640
  height: 480
  fps: 30
  depth_units: 0.001  # meters

# YOLOv8 Model Settings
model:
  path: "yolov8n.pt"
  confidence_threshold: 0.5
  iou_threshold: 0.4
  target_classes: ["bottle", "cup", "person", "trash"]

# Servo Control Settings
servos:
  pan:
    pin: 9
    range: [-90, 90]
    center: 0
  tilt:
    pin: 10
    range: [-45, 45]
    center: 0
  gripper:
    pin: 11
    range: [0, 180]
    open: 90
    closed: 0

# Control Parameters
control:
  target_distance: 0.5      # meters
  grab_threshold: 0.1       # meters
  tracking_gain: 0.1        # proportional control gain
  max_speed: 50             # maximum servo speed

# Serial Communication
serial:
  port: "/dev/ttyUSB0"
  baudrate: 9600
  timeout: 1

# Safety Settings
safety:
  max_depth: 10.0           # meters
  emergency_surface: true
  collision_avoidance: true
"""

# Create testing script
test_script = """
import cv2
import numpy as np
from rov_vision_system import ROVVisionSystem
import time

class ROVTester:
    def __init__(self):
        self.rov_system = ROVVisionSystem()
        
    def test_camera_feed(self):
        \"\"\"Test camera feed without object detection\"\"\"
        print("Testing camera feed...")
        
        import pyrealsense2 as rs
        
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        pipeline.start(config)
        
        try:
            while True:
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    continue
                
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                
                # Convert depth to colormap
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
                )
                
                # Stack images horizontally
                images = np.hstack((color_image, depth_colormap))
                
                cv2.imshow('ROV Camera Test', images)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        finally:
            pipeline.stop()
            cv2.destroyAllWindows()
            
    def test_object_detection(self):
        \"\"\"Test YOLOv8 object detection\"\"\"
        print("Testing object detection...")
        
        from ultralytics import YOLO
        
        model = YOLO("yolov8n.pt")
        
        # Test with webcam
        cap = cv2.VideoCapture(0)
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Run detection
            results = model(frame)
            
            # Draw results
            annotated_frame = results[0].plot()
            
            cv2.imshow('Object Detection Test', annotated_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
        
    def test_servo_communication(self):
        \"\"\"Test servo communication\"\"\"
        print("Testing servo communication...")
        
        import serial
        import json
        
        try:
            ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            
            # Test servo commands
            test_commands = [
                {'pan': 0, 'tilt': 0, 'gripper': 0},
                {'pan': 45, 'tilt': 30, 'gripper': 90},
                {'pan': -45, 'tilt': -30, 'gripper': 0},
                {'pan': 0, 'tilt': 0, 'gripper': 45},
            ]
            
            for cmd in test_commands:
                cmd_json = json.dumps(cmd)
                ser.write(f"{cmd_json}\\n".encode())
                print(f"Sent: {cmd}")
                time.sleep(2)
                
            ser.close()
            print("Servo communication test completed")
            
        except Exception as e:
            print(f"Servo communication error: {e}")
            
    def run_full_system_test(self):
        \"\"\"Run full system integration test\"\"\"
        print("Running full system test...")
        
        try:
            self.rov_system.start_system()
            
            # Run for 30 seconds
            for i in range(30):
                print(f"System running... {i+1}/30")
                time.sleep(1)
                
            self.rov_system.stop_system()
            print("Full system test completed")
            
        except Exception as e:
            print(f"System test error: {e}")
            self.rov_system.stop_system()

if __name__ == "__main__":
    tester = ROVTester()
    
    print("ROV System Tester")
    print("1. Test camera feed")
    print("2. Test object detection")
    print("3. Test servo communication")
    print("4. Run full system test")
    
    choice = input("Enter choice (1-4): ")
    
    if choice == '1':
        tester.test_camera_feed()
    elif choice == '2':
        tester.test_object_detection()
    elif choice == '3':
        tester.test_servo_communication()
    elif choice == '4':
        tester.run_full_system_test()
    else:
        print("Invalid choice")
"""

# Save all the support files
with open('requirements.txt', 'w') as f:
    f.write(requirements_txt)

with open('setup.sh', 'w') as f:
    f.write(setup_script)

with open('calibration.py', 'w') as f:
    f.write(calibration_script)

with open('config.yaml', 'w') as f:
    f.write(config_yaml)

with open('test_system.py', 'w') as f:
    f.write(test_script)

print("Support files created successfully!")
print("Files created:")
print("- requirements.txt")
print("- setup.sh")
print("- calibration.py")
print("- config.yaml")
print("- test_system.py")
