
add_executable(create_correspondences create_correspondences.cpp)
target_link_libraries(create_correspondences ${OpenCV_LIBS} ${RSI_LIBRARIES})