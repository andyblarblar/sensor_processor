# sensor_processor
# File
`./src/sensor_processor.cpp`

## Summary 
 Node that republishes sensor topics with a corrected frame id. Fixes caused by using ign gazebo with rviz2.
The topics published by this node are to be used only when visualising, they will fail if used with a node.

## Topics

### Publishes
- `/camera/unfiltered_image_raw`: Just a plain republish of image_raw with its frame_id set to "camera_link".
- `/robot/imu`: Just a plain republish of imu with its frame_id set to "imu_link".

### Subscribes
- `/camera/image_raw`: The raw image (not-debayerd) from the camera node/gazebo. Remap this to whatever camera you want to correct.
- `/imu`: Standard IMU data. Remap this to whatever imu you want to correct.

## Potential Improvements
Include more sensors, modularise frame_ids. 

