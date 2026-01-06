import pyrealsense2 as rs
import numpy as np
import cv2
import time

# --------------------
# Settings
# --------------------
DIST_STOP_CM = 20
DIST_SLOW_CM = 30
DIST_OBSTACLE_CM = 30
SMALL_OBSTACLE_CM = 40    
HFOV_DEG = 87.0
NUM_SLICES = 120

ALPHA = 0.6
MAX_SPEED = 100  # 0-100%

# --------------------
# RealSense setup
# --------------------
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)

depth_scale = pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()

print("Smart obstacle avoidance started")
print("Press Q to exit")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        depth_cm = depth_image * depth_scale * 100
        h, w = depth_cm.shape

        slice_width = w // NUM_SLICES
        slice_distances = []

        # --------------------
        # Compute min distance per slice
        # --------------------
        for i in range(NUM_SLICES):
            x1 = i * slice_width
            x2 = x1 + slice_width
            slice_roi = depth_cm[:, x1:x2]
            valid = slice_roi[slice_roi > 0]
            min_dist = np.min(valid) if len(valid) > 0 else np.inf
            slice_distances.append(min_dist)
        slice_distances = np.array(slice_distances)
        global_min_dist = np.min(slice_distances)
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

        # --------------------
        # Classify obstacles
        # --------------------
        big_obstacle = (depth_cm > 0) & (depth_cm < DIST_OBSTACLE_CM)
        small_obstacle = (depth_cm >= DIST_OBSTACLE_CM) & (depth_cm < SMALL_OBSTACLE_CM)
        free_space = depth_cm >= SMALL_OBSTACLE_CM

        # --------------------
        # Decision & Steering
        # --------------------
        if global_min_dist < DIST_STOP_CM:
            decision_text = "STOP"
            steering_angle = 0.0
            speed = 0
        else:
            # Weighted steering: slices with distance > 0
            valid_slices = np.copy(slice_distances)
            valid_slices[np.isinf(valid_slices)] = 0

            if np.max(valid_slices) == 0:
                steering_angle = 0.0
            else:
                # smooth weighted average for steering
                slice_indices = np.arange(NUM_SLICES)
                weights = valid_slices
                weighted_center = np.average(slice_indices, weights=weights)
                slice_center_pixel = (weighted_center + 0.5) * slice_width
                image_center = w / 2
                steering_angle = (slice_center_pixel - image_center) / w * HFOV_DEG

                # Αν είναι κοντά στο κέντρο, steer = 0
                if abs(slice_center_pixel - image_center) < slice_width:
                    steering_angle = 0.0

            steering_angle = round(steering_angle, 1)
            decision_text = f"STEER {steering_angle} deg"

            # Adaptive speed
            if global_min_dist < DIST_SLOW_CM:
                speed = int(MAX_SPEED * (global_min_dist - DIST_STOP_CM) / (DIST_SLOW_CM - DIST_STOP_CM))
                speed = max(speed, 0)
            else:
                speed = MAX_SPEED

        # --------------------
        # Visualization (drivable map)
        # --------------------
        drivable_overlay = np.zeros((h, w, 3), dtype=np.uint8)
        drivable_overlay[free_space] = (0, 255, 0)       # green
        drivable_overlay[small_obstacle] = (0, 255, 255) # yellow
        drivable_overlay[big_obstacle] = (0, 0, 255)     # red

        output = cv2.addWeighted(color_image, 1 - ALPHA, drivable_overlay, ALPHA, 0)

        # Draw steering direction
        if decision_text != "STOP":
            center_x = int(slice_center_pixel)
            cv2.line(output, (center_x, h), (w // 2, h), (255, 255, 0), 3)

        # Text overlay
        cv2.putText(output, decision_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
        cv2.putText(output, f"SPEED: {speed}%", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        cv2.putText(output, timestamp, (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Smart Obstacle Avoidance", output)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
