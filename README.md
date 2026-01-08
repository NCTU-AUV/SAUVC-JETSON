# SAUVC-JETSON
This repository should contain all the codes that need to be run on the Jetson Orin NX of Orca AUV.

## Quick Start

1. Follow steps in [getting_started](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/index.html) to setup your workspace for isaac ros 3.2</br>
   Go through "Compute Setup" and "Developer Environment Setup", mind your platform (x86/jetson)
2. ```
   cd $ISAAC_ROS_WS/src
   git clone --recurse-submodules git@github.com:NCTU-AUV/SAUVC-JETSON.git
   ```
3. Set alias for convenience. Afterward, to access a new shell into the same container, just open a new terminal and run `isa`
   ```
   echo "alias isa='cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && ./scripts/run_dev.sh' >> ~/.bashrc
   source ~/.bashrc
   isa
   ```
   This build the required image and enter bash, take a long time (40 min on my laptop)
4. ```
   cd $ISAAC_ROS_WS
   colcon build --symlink_install
   source install/setup/bash
   ```

## Demo

### Gazebo Simulation
- Launch the environment</br>
`ros2 launch sauvc_sim sauvc25_launch.py`
  
- Keyboard control and video capturing</br>
`ros2 run sauvc_sim teleop25.py`

### Yolov8 Object Detection
1. Modify line 58 in `isaac_ros_yolov8/src/yolov8_decoder_node.cpp` where # of classes is hard coded, see [issue 32](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection/issues/32#issuecomment-1827859460).
2. If you the correct class name displayed in visualization, change the `name` dict in `isaac_ros_yolov8/scripts/isaac_ros_yolov8_visualizer.py`. e.g.
   ```
   names = {
   
   		0: 'blue_drum',
   		1: 'blue_flare',
   		2: 'gate',
   		3: 'metal_ball',
   		4: 'orange_flare',
   		5: 'red_fare',
   		6: 'yellow_flare',
   
   }
   ```
4. Lauch the detection node for gazebo with visualization. For more details, refer to the [hackmd](https://hackmd.io/@NCTU-auv/H1dTKRoe6)
   ```
     cd /workspaces/isaac_ros-dev && \
      ros2 launch isaac_ros_yolov8 isaac_ros_yolov8_visualize_gazebo.launch.py \
          model_file_path:=./src/sim_best.onnx \
          engine_file_path:=./src/sim_best.plan \
          input_binding_names:=['images'] \
          output_binding_names:=['output0'] \
          network_image_width:=640 \
          network_image_height:=640 \
          force_engine_update:=False \
          image_mean:=[0.0,0.0,0.0] \
          image_stddev:=[1.0,1.0,1.0] \
          input_image_width:=640 \
          input_image_height:=640 \
          confidence_threshold:=0.25 \
          nms_threshold:=0.45
   ```

### Depth Perception
- Enable visualization for debug (default)</br>
`ros2 launch depth_perception depth_estimation.launch.py use_viz:=true`

- Disable visualization for competition runs</br>
`ros2 launch depth_perception depth_estimation.launch.py use_viz:=false`

### fsm_decision
1. Run the decision node</br>
   `ros2 run fsm_decision fsm_node`
2. Send the start signal</br>
   `ros2 topic pub --once /fsm/start_mission std_msgs/Bool "{data: true}"`
