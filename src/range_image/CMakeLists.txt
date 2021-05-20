add_executable(
    gen_range_img 
        GenRangeImg.cc
)
target_link_libraries(gen_range_img algorithm_utils ${PCL_LIBRARIES})