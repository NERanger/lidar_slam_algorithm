add_library(
    algorithm_utils
        KittiLoader.cc
        Timer.cc
)

target_link_libraries(algorithm_utils ${PCL_LIBRARIES})