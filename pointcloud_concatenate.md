# pointcloud_concatenate
# File
`./src/pointcloud.cpp`

## Summary 
 Node that combines two pointcloud topics. This is designed for use with [white line detection](https://github.com/iscumd/white_line_detection)
to combine the camera and LiDAR pointclouds.

## Topics

### Publishes
- `/combined/points`: The combined pointclouds.

### Subscribes
- `/lidar/points`: The first pointcloud topic. Traditionally LiDAR points.
- `/camera/points`: The second point source that gets translated. Traditionally camera points.

## Params
- `camera_trans_source`: The source frame of the tf transform to apply to /camera/points. Default base_footprint.
- `camera_trans_dest`: The destination frame of the tf transform to apply to /camera/points. Default laser_link.

## Potential Improvements
Make more generic, very attached to the camera rn. 

