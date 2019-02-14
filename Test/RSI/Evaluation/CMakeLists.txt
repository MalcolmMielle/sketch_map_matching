# add_executable(create_correspondences create_correspondences.cpp)
# target_link_libraries(create_correspondences ${OpenCV_LIBS} ${RSI_LIBRARIES})
# 
# add_executable(create_all_gt create_all_gt.cpp)
# target_link_libraries(create_all_gt ${OpenCV_LIBS} ${RSI_LIBRARIES} -lboost_unit_test_framework -lstdc++fs)
# 
add_executable(create_gt_from_model create_gt_from_model.cpp)
target_link_libraries(create_gt_from_model ${OpenCV_LIBS} ${RSI_LIBRARIES} -lboost_unit_test_framework -lstdc++fs)
# 
# add_executable(read_from_file read_from_file.cpp)
# target_link_libraries(read_from_file ${OpenCV_LIBS} ${RSI_LIBRARIES} -lboost_unit_test_framework)
# 
# 
# add_executable(evaluate evaluate.cpp)
# target_link_libraries(evaluate ${OpenCV_LIBS} ${RSI_LIBRARIES} -lstdc++fs  GraphLaplacianLib Comparatorlib)
# 
# add_executable(evaluate_find_time evaluate_find_time.cpp)
# target_link_libraries(evaluate_find_time ${OpenCV_LIBS} ${RSI_LIBRARIES} -lstdc++fs  GraphLaplacianLib Comparatorlib VFLib)
# 
add_executable(evaluate_multiple_times evaluate_multple_times.cpp)
target_link_libraries(evaluate_multiple_times ${OpenCV_LIBS} ${RSI_LIBRARIES} -lstdc++fs  GraphLaplacianLib Comparatorlib VFLib)
# 
# add_executable(evaluate_final_strategy evaluate_final_strategy.cpp)
# target_link_libraries(evaluate_final_strategy ${OpenCV_LIBS} ${RSI_LIBRARIES} -lstdc++fs  GraphLaplacianLib Comparatorlib VFLib)
# 
# add_executable(evaluate_threshold evaluate_threshold.cpp)
# target_link_libraries(evaluate_threshold ${OpenCV_LIBS} ${RSI_LIBRARIES} -lstdc++fs  GraphLaplacianLib Comparatorlib VFLib)
# 
# 
# add_executable(test_affine_transformation test_affine_transformation.cpp)
# target_link_libraries(test_affine_transformation ${OpenCV_LIBS} ${RSI_LIBRARIES} -lstdc++fs  GraphLaplacianLib Comparatorlib VFLib)
