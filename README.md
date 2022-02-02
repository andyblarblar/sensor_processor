# sensor_processor
# File
`./src/sensor_processor.cpp`

## Summary 
 Node that republishes sensor topics with a corrected frame id. Fixes issues where base_footprint and 
base link are not properly connected, causing sensor topics to contain invalid frame ids.

## Topics

### Publishes
- `/camera/unfiltered_image_raw`: Just a plain republish of image_raw with its frame_id set to "camera_link".

### Subscribes
- `/camera/image_raw`: The raw image (not-debayerd) from the camera node/gazebo. Remap this to whatever camera you want to correct.

## Potential Improvements
Investigate modularising how sensors are enabled, and include more sensors. 

# Launch 
 `TBD` 
 standard launch file, just make sure to remap all topics to what you need for your bot. 

