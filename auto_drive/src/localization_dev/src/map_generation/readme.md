### build

`cd PATH_TO_map_generation/build`
`cmake ..`
`make`

### execute

`cd PATH_TO_map_generation/build`
`./map_generator`

### information
#### .pcd file requred in /mesh
#### use `pcl_mesh_sampling input.obj  output.pcd -leaf_size 0.01 -no_vis_result` cmd to generate .pcd file from .obj file
#### creating .obj file with meshlab is recommended