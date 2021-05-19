add_executable(
    passthrough_filter 
        PassThroughFilter.cc
)
target_link_libraries(passthrough_filter algorithm_utils ${PCL_LIBRARIES})