# requrements
- nav2_map_server
- nav2_util
- do `colcon build` and `source ./install/setup.bash`

# usage

## save map from octmap topic using this cmd

`ros2 run nav2_map_server map_saver_cli -t <topic_name> -f <map_name>`

## publish octmap from map in PATH_TO_PKG/maps using this cmd

`ros2 launch map_server.launch.py`

- map name and directry are changable by setting "map_file_path" in launch/map_server.launch.py