# localization development environment package

# requrement
- do `colcon build` and `source ./install/setup.bash`

## usage
### set parameter
- user can set parameters by editting /params/params.yaml in this pkg

- user can set start(x,y), goal(x,y), robot_size(Length of a side of a square) and the path that planned path will be saved

### path planning
- launch path planner `ros2 launch global_path global_path_test.launch.py`

- the global path will be automatically planned and saved

- the map used to local path planning is loaded from /map directry in this pkg
