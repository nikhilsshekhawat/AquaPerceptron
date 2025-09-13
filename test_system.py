import cv2
import numpy as np
from rov_vision_system import ROVVisionSystem
import time

class ROVTester:
    def __init__(self):
        self.rov_system = ROVVisionSystem()

    def test_camera_feed(self):
        """Test camera feed without object detection"""
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
        """Test YOLOv8 object detection"""
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
        """Test servo communication"""
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
                ser.write(f"{cmd_json}\n".encode())
                print(f"Sent: {cmd}")
                time.sleep(2)

            ser.close()
            print("Servo communication test completed")

        except Exception as e:
            print(f"Servo communication error: {e}")

    def run_full_system_test(self):
        """Run full system integration test"""
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
