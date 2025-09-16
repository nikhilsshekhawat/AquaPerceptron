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
        """Calibrate the RealSense camera"""
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
        """Test depth measurement accuracy"""
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
        """Calibrate servo movement range"""
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
