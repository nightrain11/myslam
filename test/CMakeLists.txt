add_executable( run_vo run_vo.cpp )
target_link_libraries( run_vo myslam ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})