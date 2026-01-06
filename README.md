# Smart Obstacle Avoidance with Intel RealSense

This project is a **smart obstacle avoidance system** using the **Intel RealSense** camera. It processes depth and color images to detect obstacles in real time and decides **steering direction** and **speed** for safe navigation of a robot or vehicle.

---

## 🔹 Features

- **Slice-based depth analysis**: The camera image is divided into vertical slices to compute the closest obstacle in each region.
- **Obstacle classification**:
  - **Big obstacles**: very close → `STOP`
  - **Small obstacles**: near → reduce speed, caution
  - **Free space**: safe area → green overlay
- **Steering angle calculation**: Weighted average of slice distances determines the direction to move.
- **Adaptive speed control**: Slows down as obstacles approach the robot.
- **Real-time visualization**: Color image overlayed with obstacle map, steering direction, and speed.
- Timestamp display for debugging and logging.

---

## 🔹 How It Works

1. **Depth Image Slicing**  
   The depth image is divided into `NUM_SLICES` vertical slices. For each slice, the system computes the **minimum distance** to any obstacle.

2. **Obstacle Classification**  
   - **Big obstacles**: distance < `DIST_OBSTACLE_CM` → red  
   - **Small obstacles**: `DIST_OBSTACLE_CM <= distance < SMALL_OBSTACLE_CM` → yellow  
   - **Free space**: distance >= `SMALL_OBSTACLE_CM` → green  

3. **Steering Calculation**  
   - The system calculates a **weighted average of slices** based on their distance.  
   - The closer the obstacle, the lower the weight.  
   - The **steering angle** (in degrees) is calculated relative to the center of the camera's horizontal field of view (HFOV).  
   - Formula (simplified):
     ```
     steering_angle = (slice_center_pixel - image_center) / image_width * HFOV_DEG
     ```
   - If the best path is near the center, the steering angle is 0.

4. **Adaptive Speed Control**
   - **Full stop**: `global_min_dist < DIST_STOP_CM` → speed = 0  
   - **Slow down**: `DIST_STOP_CM <= global_min_dist < DIST_SLOW_CM` → speed scaled linearly:  
     ```
     speed = MAX_SPEED * (global_min_dist - DIST_STOP_CM) / (DIST_SLOW_CM - DIST_STOP_CM)
     ```
   - **Maximum speed**: `global_min_dist >= DIST_SLOW_CM` → speed = `MAX_SPEED`  

---

## 🔹 Parameters

All parameters can be modified at the top of the script:

| Parameter            | Description |
|----------------------|-------------|
| `DIST_STOP_CM`       | Distance at which the robot must **stop** (cm) |
| `DIST_SLOW_CM`       | Distance at which the robot **starts slowing down** (cm) |
| `DIST_OBSTACLE_CM`   | Distance threshold for **big obstacles** (cm) |
| `SMALL_OBSTACLE_CM`  | Distance threshold for **small obstacles** (cm) |
| `HFOV_DEG`           | Camera **Horizontal Field of View** (degrees) |
| `NUM_SLICES`         | Number of vertical slices for depth analysis |
| `MAX_SPEED`          | Maximum speed (%) |
| `ALPHA`              | Transparency of the overlay in visualization (0–1) |

---

## 🔹 Visualization

The program overlays a **drivable map** on the color image:

- **Red**: big obstacle (stop immediately)  
- **Yellow**: small obstacle (slow down)  
- **Green**: free space (safe to move)  
- **Steering direction**: line from bottom center toward the chosen slice  
- **Text overlay**: `STOP` or `STEER xx deg` and speed in %  
- **Timestamp**: for debugging

---

## 🔹 Requirements

- Python 3.8+
- [pyrealsense2](https://pypi.org/project/pyrealsense2/)
- OpenCV (`opencv-python`)
- NumPy

Install requirements with:

```bash
pip install pyrealsense2 opencv-python numpy
