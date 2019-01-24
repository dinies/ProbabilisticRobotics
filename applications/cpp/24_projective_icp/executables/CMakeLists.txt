#ds camera_test: projection test
add_executable(camera_test camera_test.cpp 
  ../src/utils.cpp
  ../src/points_utils.cpp
  ../src/camera.cpp
)
target_link_libraries(camera_test opencv_highgui)

#ds correspondence_finder_test: association finding test
add_executable(correspondence_finder_test correspondence_finder_test.cpp 
  ../src/utils.cpp
  ../src/points_utils.cpp
  ../src/camera.cpp
  ../src/distance_map.cpp
  ../src/distance_map_correspondence_finder.cpp
)
target_link_libraries(correspondence_finder_test opencv_highgui)

#ds picp_solver_test: projective icp with known associations
add_executable(picp_solver_test picp_solver_test.cpp 
  ../src/utils.cpp
  ../src/points_utils.cpp
  ../src/camera.cpp
  ../src/picp_solver.cpp 
)
target_link_libraries(picp_solver_test opencv_highgui)

#ds picp_complete_test: projective icp using the distance map
add_executable(picp_complete_test picp_complete_test.cpp
  ../src/utils.cpp
  ../src/points_utils.cpp
  ../src/camera.cpp
  ../src/distance_map.cpp
  ../src/distance_map_correspondence_finder.cpp
  ../src/picp_solver.cpp 
)
target_link_libraries(picp_complete_test opencv_highgui)
