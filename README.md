# Smart Obstacle Avoidance with Intel RealSense

This project is a **smart obstacle avoidance system** using the **Intel RealSense** camera. The system processes depth and color images to detect obstacles, calculate the best path, and adjust the speed of a robot or vehicle.

---

## 🔹 Features

- **Slice-based depth analysis** for obstacle detection.
- **Obstacle classification**:
  - **Big obstacles**: very close → STOP
  - **Small obstacles**: near → caution / reduced speed
  - **Free space**: safe area → green
- **Steering angle calculation** based on available slices.
- **Adaptive speed control** depending on distance to obstacles.
- **Real-time visualization** overlay on color image.
- Displays decision text, speed, and timestamp.

---

## 🔹 Requirements

- Python 3.8+
- [pyrealsense2](https://pypi.org/project/pyrealsense2/)
- OpenCV (`opencv-python`)
- NumPy

Install requirements with:

```bash
pip install pyrealsense2 opencv-python numpy
