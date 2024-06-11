# requrements
- nav2_map_server
- nav2_util
- nav2_amcl
- holonimic_sim
- do `colcon build` and `source ./install/setup.bash`

# usage

- launch localization test

`ros2 launch localization_bynav2 nav2_amcl.launch.py`

- you can see accuracy of localization on xterm console