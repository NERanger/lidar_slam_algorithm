add_executable(
    passthrough_filter 
        PassThroughFilter.cc
)
target_link_libraries(passthrough_filter algorithm_utils ${PCL_LIBRARIES})

add_executable(
    voxel_grid_filter 
        VoxelGridFilter.cc
)
target_link_libraries(voxel_grid_filter algorithm_utils ${PCL_LIBRARIES})