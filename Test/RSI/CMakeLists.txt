add_executable(test_conversion_rsi test_conversion.cpp)
target_link_libraries(test_conversion_rsi ${OpenCV_LIBS} ${RSI_LIBRARIES} Comparatorlib -lboost_filesystem -lboost_system -lboost_unit_test_framework)

add_executable(match_anchors match_anchors.cpp)
target_link_libraries(match_anchors ${OpenCV_LIBS} ${RSI_LIBRARIES} Comparatorlib -lboost_filesystem -lboost_system -lboost_unit_test_framework)


add_executable(test_shjared_ptr test_shjared_ptr.cpp)
target_link_libraries(test_shjared_ptr ${OpenCV_LIBS} ${RSI_LIBRARIES} Comparatorlib -lboost_filesystem -lboost_system -lboost_unit_test_framework)