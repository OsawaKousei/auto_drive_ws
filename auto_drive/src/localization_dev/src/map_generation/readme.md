# requrement
- build map geberator

`cd PATH_TO_map_generation/build`

`cmake ..`

`make`

# usage
## map generation
- use this cmd

`cd PATH_TO_map_generation/build`

`./map_generator`

- this cmd genarate .pcd format point cloud map as map_generation/map/map.pcd from map_generation/meshes/output.pcd

## generation parameter setting
- generation function is implemented in map_generation/src/map_generator.cpp
- user can change paramter for map generation

    see comments in map_generator.cpp for details

- if you made any change in map_generator.cpp, rebuild to refrection changes

## make .pcd file
-  user can make .pcd file by following these steps

    create .obj or .ply file you want to use

    meshlab serves conversion to .obj from various CAD format

    use `pcl_mesh_sampling input.obj  output.pcd -leaf_size 0.01 -no_vis_result` cmd to generate .pcd file from .obj file

- leaf_size argument is changable