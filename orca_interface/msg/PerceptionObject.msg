# PerceptionObject.msg
# Single detected object, output by the depth_perception node.

string label           # English class name (mapped from class_id via names[] param)
float32 confidence     # Detection confidence score [0.0, 1.0]

# --- 2D bounding box (in YOLO input pixel coordinates) ---
float32 cx             # Bounding box centre x
float32 cy             # Bounding box centre y
float32 width          # Bounding box width
float32 height         # Bounding box height

# --- 3D pose relative to AUV (RealSense mode only) ---
# pose.position = (X, Y, Z) in metres; X right, Y down, Z forward (camera frame)
# pose.orientation = identity quaternion when not solved
geometry_msgs/Pose pose

# --- Range ---
# Distance to object in metres; -1.0 if unknown (USB mode or depth invalid)
float32 distance

# --- Single-frame depth sanity flag ---
# True if the depth estimate for this frame passed internal consistency checks
bool valid

# --- Multi-frame stability flag ---
# True when:
#   detected_in_last_N_frames >= stability_min_hits
#   AND std_dev(pose.position) over window < stability_pos_std_thresh
#   AND confidence >= stability_conf_thresh
bool is_stable
