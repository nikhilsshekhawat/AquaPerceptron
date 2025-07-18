
# ROV Vision System with YOLOv8 and Intel RealSense

## Overview

This system integrates YOLOv8 object detection with Intel RealSense depth cameras to create an autonomous ROV (Remotely Operated Vehicle) capable of detecting, tracking, and manipulating objects underwater. The system uses real-time computer vision to identify objects, calculate their 3D coordinates, and control servo-mounted grippers to grab objects and place them in designated locations.

## System Architecture

### Hardware Components

1. **Intel RealSense Camera (D435/D435i)**
   - Provides RGB and depth data
   - 640x480 resolution at 30 FPS
   - Depth range: 0.3m to 3m (ideal for ROV operations)

2. **Servo Motors**
   - Pan Servo: Controls horizontal camera/gripper movement
   - Tilt Servo: Controls vertical camera/gripper movement
   - Gripper Servo: Controls gripper open/close mechanism

3. **Arduino Microcontroller**
   - Receives servo commands via serial communication
   - Controls PWM signals to servo motors
   - Provides real-time servo positioning

4. **ROV Platform**
   - Waterproof housing for electronics
   - Stable platform for camera and gripper mounting
   - Tether cable for communication and power

### Software Components

1. **Main Vision System (rov_vision_system.py)**
   - Multi-threaded architecture for real-time processing
   - YOLOv8 integration for object detection
   - RealSense camera interface
   - Servo control system
   - Coordinate transformation algorithms

2. **Calibration System (calibration.py)**
   - Camera intrinsic parameter calibration
   - Depth accuracy testing
   - Servo range calibration

3. **Testing Framework (test_system.py)**
   - Individual component testing
   - Integration testing
   - Performance monitoring

## Key Features

### Real-Time Object Detection
- Uses YOLOv8 for fast, accurate object detection
- Supports multiple object classes (customizable)
- Confidence-based filtering for reliable detection

### 3D Coordinate Calculation
- Combines RGB detection with depth data
- Pixel-to-world coordinate transformation
- Real-time distance measurement

### Servo Control System
- Proportional control for smooth tracking
- Safety limits and bounds checking
- JSON-based command protocol

### Multi-Threading Architecture
- Separate threads for:
  - Frame capture (RealSense)
  - Object detection (YOLOv8)
  - Servo control
  - Main coordination

## Installation and Setup

### Prerequisites
- Python 3.8+
- Intel RealSense SDK 2.0
- Arduino IDE
- Ubuntu 20.04+ (recommended)

### Installation Steps

1. **Clone the repository and install dependencies:**
```bash
git clone <repository-url>
cd rov-vision-system
chmod +x setup.sh
./setup.sh
```

2. **Upload Arduino code to microcontroller:**
   - Open Arduino IDE
   - Copy the servo controller code from `ServoController.get_arduino_code()`
   - Install ArduinoJson library
   - Upload to Arduino

3. **Calibrate the system:**
```bash
python calibration.py
```

4. **Test individual components:**
```bash
python test_system.py
```

5. **Run the main system:**
```bash
python rov_vision_system.py
```

## Configuration

### Camera Settings
- Resolution: 640x480 (adjustable)
- Frame rate: 30 FPS
- Depth units: 0.001m (1mm precision)

### Detection Parameters
- Confidence threshold: 0.5
- IoU threshold: 0.4
- Target classes: configurable in config.yaml

### Control Parameters
- Target distance: 0.5m
- Grab threshold: 0.1m
- Tracking gain: 0.1 (proportional control)

## Usage

### Basic Operation
1. Start the system: `python rov_vision_system.py`
2. The system will automatically:
   - Initialize camera and servos
   - Start object detection
   - Track detected objects
   - Grab objects when close enough
   - Place objects in designated location

### Manual Control
- Modify `config.yaml` for different target objects
- Adjust servo limits and control gains
- Configure serial communication parameters

## System Workflow

1. **Frame Capture**: RealSense camera captures RGB and depth frames
2. **Object Detection**: YOLOv8 processes RGB frames to detect objects
3. **3D Localization**: Depth data is used to calculate object positions
4. **Servo Control**: Pan/tilt servos track the target object
5. **Grabbing**: When object is within threshold, gripper activates
6. **Placement**: Object is moved to designated dustbin location

## Performance Characteristics

### Processing Speed
- YOLOv8n: ~30 FPS on modern hardware
- RealSense: 30 FPS RGB+Depth
- Servo response: ~20ms

### Accuracy
- Object detection: >90% for well-lit conditions
- Depth accuracy: ±2% at 0.5m distance
- Servo positioning: ±1 degree

## Troubleshooting

### Common Issues

1. **Camera not detected**
   - Check USB connection
   - Verify RealSense SDK installation
   - Run `realsense-viewer` to test

2. **Servo not responding**
   - Check serial port connection
   - Verify Arduino code upload
   - Test with `test_system.py`

3. **Poor detection accuracy**
   - Improve lighting conditions
   - Adjust confidence threshold
   - Retrain model with custom data

### Debug Mode
Enable debug logging by setting `DEBUG=True` in the main script.

## Customization

### Adding New Object Classes
1. Collect training data
2. Annotate using tools like Roboflow
3. Train custom YOLOv8 model
4. Update model path in config

### Modifying Servo Behavior
1. Adjust control gains in config.yaml
2. Modify servo limits for different hardware
3. Implement custom grabbing sequences

## Safety Considerations

### Underwater Operations
- Ensure waterproof sealing of all electronics
- Test depth limits before deployment
- Include emergency surfacing capability

### Servo Safety
- Implement position limits
- Include emergency stop functionality
- Monitor servo current draw

## Future Enhancements

### Planned Features
- Multi-object tracking
- Obstacle avoidance
- Advanced path planning
- Remote monitoring dashboard

### Integration Possibilities
- ROS integration
- Web-based control interface
- Machine learning improvements
- Acoustic positioning

## API Reference

### Main Classes

#### ROVVisionSystem
- `start_system()`: Initialize and start all components
- `stop_system()`: Gracefully shutdown system
- `_capture_frames()`: Camera frame capture thread
- `_detect_objects()`: Object detection thread
- `_control_servos()`: Servo control thread

#### ServoController
- `get_arduino_code()`: Returns Arduino servo control code

### Configuration Files

#### config.yaml
Main configuration file containing all system parameters.

#### camera_calibration.json
Camera intrinsic parameters from calibration process.

#### servo_calibration.json
Servo movement ranges and limits.

## Contributing

1. Fork the repository
2. Create feature branch
3. Make changes with proper testing
4. Submit pull request

## License

This project is licensed under the MIT License - see LICENSE file for details.

## Acknowledgments

- Ultralytics for YOLOv8
- Intel for RealSense SDK
- OpenCV community
- ROV research community

## Support

For issues and questions:
1. Check troubleshooting section
2. Review existing issues
3. Create new issue with detailed description
4. Include system specifications and error logs
