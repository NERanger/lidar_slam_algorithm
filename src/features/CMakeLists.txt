add_executable(
    normal_extraction 
        NormalExtraction.cc
)
target_link_libraries(normal_extraction algorithm_utils ${PCL_LIBRARIES})

add_executable(
    pfh_extraction
        PFHExtraction.cc   
)
target_link_libraries(pfh_extraction algorithm_utils ${PCL_LIBRARIES})

add_executable(
    fpfh_extraction
        FPFHExtraction.cc
)
target_link_libraries(fpfh_extraction algorithm_utils ${PCL_LIBRARIES})

add_executable(
    vfh_extraction
        VFHExtraction.cc
)
target_link_libraries(vfh_extraction algorithm_utils ${PCL_LIBRARIES})