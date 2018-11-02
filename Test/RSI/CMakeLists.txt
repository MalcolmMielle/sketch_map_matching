add_executable(test_conversion_rsi test_conversion.cpp)
target_link_libraries(test_conversion_rsi ${OpenCV_LIBS} ${RSI_LIBRARIES} Comparatorlib -lboost_filesystem -lboost_system -lboost_unit_test_framework)

add_executable(match_anchors match_anchors.cpp)
target_link_libraries(match_anchors ${OpenCV_LIBS} ${RSI_LIBRARIES} GraphLaplacianLib Comparatorlib -lboost_filesystem -lboost_system -lboost_unit_test_framework)


add_executable(test_shjared_ptr test_shjared_ptr.cpp)
target_link_libraries(test_shjared_ptr ${OpenCV_LIBS} ${RSI_LIBRARIES} Comparatorlib -lboost_filesystem -lboost_system -lboost_unit_test_framework)


add_executable(test_laplacian test_laplacian.cpp)
target_link_libraries(test_laplacian ${OpenCV_LIBS} ${RSI_LIBRARIES} GraphLaplacianLib -lboost_filesystem -lboost_system -lboost_unit_test_framework)

add_executable(test_vfl test_vfl.cpp)
target_link_libraries(test_vfl ${OpenCV_LIBS} ${RSI_LIBRARIES} GraphLaplacianLib VFLib Comparatorlib)

add_executable(test_hungarian test_hungarian.cpp)
target_link_libraries(test_hungarian ${OpenCV_LIBS} ${RSI_LIBRARIES} GraphLaplacianLib Comparatorlib -lboost_filesystem -lboost_system -lboost_unit_test_framework)