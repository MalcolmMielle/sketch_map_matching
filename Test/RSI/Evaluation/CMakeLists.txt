add_executable(create_correspondences create_correspondences.cpp)
target_link_libraries(create_correspondences ${OpenCV_LIBS} ${RSI_LIBRARIES})

add_executable(create_all_gt create_all_gt.cpp)
target_link_libraries(create_all_gt ${OpenCV_LIBS} ${RSI_LIBRARIES} -lboost_unit_test_framework -lstdc++fs)

add_executable(read_from_file read_from_file.cpp)
target_link_libraries(read_from_file ${OpenCV_LIBS} ${RSI_LIBRARIES} -lboost_unit_test_framework)