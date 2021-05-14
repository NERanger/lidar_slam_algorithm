add_executable(
    normal_extraction 
        NormalExtraction.cc
)
target_link_libraries(normal_extraction algorithm_utils ${PCL_LIBRARIES})