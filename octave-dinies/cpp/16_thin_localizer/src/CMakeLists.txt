add_library(thin_localizer_library 
  distance_map.cpp distance_map.h
  #transition_model.cpp transition_model.h
  #observation_model.cpp observation_model.h
  localizer.cpp localizer.h
  )

target_link_libraries(thin_localizer_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
)

add_executable(thin_localizer_node
  thin_localizer_node.cpp
  )

target_link_libraries(thin_localizer_node
  thin_localizer_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
)


