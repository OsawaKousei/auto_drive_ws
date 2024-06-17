# local path planning package

# requrement
- do `colcon build` and `source ./install/setup.bash`

## usage
### set parameter
- user can set parameters by editting /params/params.yaml in this pkg

- user can set pass detection threshold, how many points of global path and how many points complete by spline

### path planning
- launch path planner `ros2 launch local_path global_path_test.launch.py`

- you can see global path as green line and local path as red line

- the global used to local path planning is loaded from /path directry in this pkg
