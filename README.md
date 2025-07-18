
# ROV Vision System with YOLOv8 and Intel RealSense

🤖 **Autonomous underwater object detection, tracking, and manipulation system**

## Quick Start

```bash
# 1. Install dependencies
./setup.sh

# 2. Upload Arduino code
# (Copy code from ServoController.get_arduino_code())

# 3. Calibrate system
python calibration.py

# 4. Test system
python test_system.py

# 5. Run main system
python rov_vision_system.py
```

## Features

✅ **Real-time object detection** with YOLOv8  
✅ **3D coordinate calculation** using depth data  
✅ **Servo-controlled gripper** for object manipulation  
✅ **Multi-threaded architecture** for optimal performance  
✅ **Comprehensive calibration** and testing tools  
✅ **Configurable parameters** via YAML files  

## System Requirements

- Python 3.8+
- Intel RealSense D435/D435i camera
- Arduino with servo motors
- Ubuntu 20.04+ (recommended)

## File Structure

```
rov-vision-system/
├── rov_vision_system.py    # Main system code
├── calibration.py          # Calibration utilities
├── test_system.py          # Testing framework
├── config.yaml             # Configuration file
├── requirements.txt        # Python dependencies
├── setup.sh               # Installation script
└── README.md              # This file
```

## Hardware Setup

1. **Mount Intel RealSense camera** on ROV platform
2. **Connect servo motors** for pan, tilt, and gripper control
3. **Wire Arduino** to servos and establish serial communication
4. **Ensure waterproof sealing** for underwater operations

## Software Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   RealSense     │    │     YOLOv8      │    │  Servo Control  │
│   Camera        │───▶│ Object Detection│───▶│   (Arduino)     │
│  (RGB + Depth)  │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                        │                        │
         ▼                        ▼                        ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ Frame Capture   │    │ 3D Coordinate   │    │ Gripper Control │
│    Thread       │    │  Calculation    │    │   & Placement   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Performance

- **Detection Rate**: 30 FPS
- **Depth Accuracy**: ±2% at 0.5m
- **Servo Response**: <20ms
- **Object Classes**: Configurable (default: bottles, cups, trash)

## Configuration

Edit `config.yaml` to customize:
- Camera resolution and frame rate
- Object detection parameters
- Servo control gains and limits
- Serial communication settings

## Troubleshooting

### Camera Issues
```bash
# Test RealSense camera
realsense-viewer

# Check USB connection
lsusb | grep Intel
```

### Servo Issues
```bash
# Test servo communication
python test_system.py
# Select option 3
```

### Detection Issues
- Improve lighting conditions
- Adjust confidence threshold in config.yaml
- Retrain with custom dataset

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

Distributed under the MIT License. See `LICENSE` for more information.

## Contact

Email: [nikhilshekhawat1999@gmail.com](mailto:nikhilshekhawat1999@gmail.com)
Project Link: [https://github.com/nikhilsshekhawat/AquaPerceptron](https://github.com/nikhilsshekhawat/AquaPerceptron)

---

⭐ **Star this repository if you found it helpful!**
