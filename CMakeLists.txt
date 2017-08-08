#for debug : cmake -DCMAKE_BUILD_TYPE=Debug /path/to/source

cmake_minimum_required(VERSION 2.6)
project(sketchAlgorithms)

add_definitions(-std=c++11)

SET(CMAKE_CXX_FLAGS "-Wall")

# Version for all the file in debug have TIMED defined in debug mode
SET(CMAKE_CXX_FLAGS_DEBUG  "-DDEBUG -g")
#Version for the same thing but for one target
# target_compile_definitions(Probabilitieslib PUBLIC $<$<CONFIG:Debug>:TIMED>)

find_package( OpenCV REQUIRED)

find_package( Boost REQUIRED )

find_package(RSI REQUIRED)

find_package( editdistance REQUIRED )
find_package( vodigrex REQUIRED )
find_package( bettergraph REQUIRED )

if ( NOT vodigrex_FOUND )
  message(STATUS "This project requires the VODIGREX library, and will not be compiled.")
  return()  
endif()


get_cmake_property(_variableNames VARIABLES)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()  
endif()


# //I don't think those are needed because they are in usr/include
# include_directories(${editdistance_INCLUDE_DIRS})
# include_directories(${vodigrex_INCLUDE_DIRS})

# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()
# install(TARGETS editdistance RUNTIME DESTINATION bin)


include_directories("${PROJECT_BINARY_DIR}" ${CMAKE_CURRENT_BINARY_DIR} ${vodigrex_INCLUDE_DIRS} ${RSI_INCLUDE_DIRS}
	includes
	includes/TopologicalMap/ 
	includes/MapComparator/ 
	Test/Ddata/ 
	includes/MapComparator/Keypoints 
	includes/ProbabilisticMethod
	includes/VFLib/include/
	includes/VFLConversion/
	includes/Conversion/
)

# add_library(EVGthin
# 			includes/Evg_thin/evg-thin.cc)

# add_library(Maplib 
# 			includes/MapInterpretation/SketchMap.cpp
# # 			includes/MapInterpretation/Thinker_Voronoi.cpp
# 			includes/MapInterpretation/Thinker_EVG.cpp)		
# target_link_libraries(Maplib ${OpenCV_LIBS} ${vodigrex_LIBRARIES} )

add_library(Comparatorlib
			includes/MapComparator/Init.cpp
			includes/MapComparator/Cluster.cpp
			includes/MapComparator/GraphMatcherBase.cpp
			includes/MapComparator/GraphMatcherClusterFiltered.cpp
			includes/MapComparator/GraphMatcherNeighbor.cpp
			includes/MapComparator/GraphMatcherOld.cpp
			includes/MapComparator/GraphMatcherAnchor.cpp
			includes/MapComparator/StorageFilter.cpp
			includes/MapComparator/Hypothese.cpp
			)
target_link_libraries(Comparatorlib)

add_library(VFLib
			includes/VFLib/src/argedit.cc
			includes/VFLib/src/argloader.cc
			includes/VFLib/src/argraph.cc
			includes/VFLib/src/error.cc
			includes/VFLib/src/gene.cc
			includes/VFLib/src/gene_mesh.cc
			includes/VFLib/src/match.cc
			includes/VFLib/src/sd_state.cc
			includes/VFLib/src/sortnodes.cc
			includes/VFLib/src/ull_state.cc
			includes/VFLib/src/ull_sub_state.cc
			includes/VFLib/src/vf2_mono_state.cc
			includes/VFLib/src/vf2_state.cc
			includes/VFLib/src/vf2_sub_state.cc
			includes/VFLib/src/vf_mono_state.cc
			includes/VFLib/src/vf_state.cc
			includes/VFLib/src/vf_sub_state.cc
			includes/VFLib/src/xsubgraph.cc
			)

# add_executable(test_graphline Test/BaseTest/test_graphline.cpp)
# target_link_libraries(test_graphline ${OpenCV_LIBS} ${vodigrex_LIBRARIES} -lboost_unit_test_framework)
# 
# add_executable(test_map_import Test/BaseTest/mapimport.cpp)
# target_link_libraries(test_map_import ${OpenCV_LIBS} ${vodigrex_LIBRARIES} -lboost_unit_test_framework Maplib)
# 
# add_executable(test_voronoi Test/BaseTest/Voronoi_test.cpp)
# target_link_libraries(test_voronoi ${OpenCV_LIBS} -lboost_unit_test_framework Maplib)
# 
# add_executable(test_delaunay Test/BaseTest/test_delaunay.cpp)
# target_link_libraries(test_delaunay ${OpenCV_LIBS} -lboost_unit_test_framework Maplib)
# 
# add_executable(test_valgrind_thinker Test/BaseTest/test_think_voronoi_valgrind.cpp)
# target_link_libraries(test_valgrind_thinker ${OpenCV_LIBS} -lboost_unit_test_framework Maplib)
# 
# add_executable(test_laplace Test/BaseTest/laplacetest.cpp)
# target_link_libraries(test_laplace ${OpenCV_LIBS} Maplib)
# 
# add_executable(test_line_follower Test/BaseTest/line_follower.cpp)
# target_link_libraries(test_line_follower ${OpenCV_LIBS} -lboost_unit_test_framework Maplib)
# 
# add_executable(test_extractor Test/BaseTest/test_extractor.cpp)
# target_link_libraries(test_extractor ${OpenCV_LIBS} -lboost_unit_test_framework Maplib)
# 
# add_executable(test_comparator Test/BaseTest/test_comparisor.cpp)
# target_link_libraries(test_comparator ${OpenCV_LIBS} -lboost_unit_test_framework Maplib Comparatorlib)
# # 
# # add_executable(test_cluster Test/BaseTest/test_cluster.cpp)
# # target_link_libraries(test_cluster ${OpenCV_LIBS} Comparatorlib -lboost_unit_test_framework)
# # 
# add_executable(test_real_comparaison Test/BaseTest/test_real_comparison.cpp)
# target_link_libraries(test_real_comparaison ${OpenCV_LIBS} Comparatorlib -lboost_unit_test_framework)
# # 
# # 
# add_executable(test_evg Test/BaseTest/test_EVG.cpp)
# target_link_libraries(test_evg ${OpenCV_LIBS} ${vodigrex_LIBRARIES} Comparatorlib -lboost_unit_test_framework)
# # 
# add_executable(test_virtual Test/BaseTest/test_virtualenv.cpp)
# target_link_libraries(test_virtual ${OpenCV_LIBS} Comparatorlib -lboost_unit_test_framework)
# # 
# add_executable(test_placevirtual Test/BaseTest/test_place.cpp)
# target_link_libraries(test_placevirtual ${OpenCV_LIBS} Comparatorlib -lboost_unit_test_framework)
# # 
# add_executable(test_updateEditDistance Test/BaseTest/test_updateEditDistance.cpp)
# target_link_libraries(test_updateEditDistance ${OpenCV_LIBS} Comparatorlib -lboost_unit_test_framework Probabilitieslib) 
# # 
# add_executable(test_randomgraph Test/BaseTest/test_randomgraph.cpp)
# target_link_libraries(test_randomgraph ${OpenCV_LIBS} Comparatorlib Maplib -lboost_unit_test_framework)
# 
# add_executable(test_randomgraph_vfl Test/BaseTest/test_randomgraph_vfl.cpp)
# target_link_libraries(test_randomgraph_vfl ${OpenCV_LIBS} Comparatorlib Maplib VFLib -lboost_unit_test_framework)
# # 
# add_executable(test_multiline Test/BaseTest/test_multilline.cpp)
# target_link_libraries(test_multiline ${OpenCV_LIBS} Maplib Comparatorlib -lboost_unit_test_framework)
# # 
# # add_executable(test_dani Test/BaseTest/Dani.cpp)
# # target_link_libraries(test_dani ${OpenCV_LIBS} Maplib Comparatorlib -lboost_unit_test_framework)
# # 
# # 
# add_executable(test_door_line Test/BaseTest/test_line_doors.cpp)
# target_link_libraries(test_door_line ${OpenCV_LIBS} -lboost_unit_test_framework Maplib Comparatorlib)
# 
# add_executable(test_anchor Test/BaseTest/test_anchor.cpp)
# target_link_libraries(test_anchor ${OpenCV_LIBS} -lboost_unit_test_framework Maplib Comparatorlib)
# 
# add_executable(test_variance Test/Probabilities/test_variance.cpp)
# target_link_libraries(test_variance ${OpenCV_LIBS} Probabilitieslib -lboost_unit_test_framework Maplib Comparatorlib)
# 
# add_executable(test_bigger_graph Test/Probabilities/test_bigger_graph.cpp)
# target_link_libraries(test_bigger_graph ${OpenCV_LIBS} Probabilitieslib -lboost_unit_test_framework Maplib Comparatorlib)
# 
# add_executable(test_hypoproba Test/Probabilities/test_hypoproba.cpp)
# target_link_libraries(test_hypoproba ${OpenCV_LIBS} Probabilitieslib -lboost_unit_test_framework Maplib Comparatorlib)
# 
# add_executable(test_table Test/Probabilities/test_table.cpp)
# target_link_libraries(test_table ${OpenCV_LIBS} Probabilitieslib -lboost_unit_test_framework Maplib Comparatorlib)
# 
# add_executable(test_anchorfind Test/Probabilities/test_anchor_finding.cpp)
# target_link_libraries(test_anchorfind ${OpenCV_LIBS} Probabilitieslib -lboost_unit_test_framework Maplib Comparatorlib)
# 
# add_executable(test_real_proba Test/Probabilities/test_real_proba.cpp)
# target_link_libraries(test_real_proba ${OpenCV_LIBS} Probabilitieslib -lboost_unit_test_framework Maplib Comparatorlib)
# target_compile_definitions(test_real_proba PUBLIC $<$<CONFIG:Debug>:TIMED>)

# 
# add_executable(test_proba_time Test/Probabilities/test_proba_time.cpp)
# target_link_libraries(test_proba_time ${OpenCV_LIBS} Probabilitieslib -lboost_unit_test_framework Maplib Comparatorlib)
# 
# add_executable(test_randomgraph_proba Test/Probabilities/test_randomgraph_proba.cpp)
# target_link_libraries(test_randomgraph_proba ${OpenCV_LIBS} Probabilitieslib -lboost_unit_test_framework Maplib Comparatorlib)
# 
# add_executable(test_catch Test/Probabilities/test_catch.cpp)
# 
# add_executable(test_nd Test/Probabilities/test_nd.cpp)
# target_link_libraries(test_nd -lboost_unit_test_framework Probabilitieslib)
# 
# 
# add_executable(test_zonesame Test/Probabilities/test_zonesame.cpp)
# target_link_libraries(test_zonesame ${OpenCV_LIBS} Comparatorlib -lboost_unit_test_framework)
# 
# add_executable(test_vflib_standard Test/Probabilities/standard_test_iso.cpp)
# target_link_libraries(test_vflib_standard ${OpenCV_LIBS} Comparatorlib -lboost_unit_test_framework VFLib)


# add_executable(test_graphdb Test/GraphDB/test_graphdb.cpp)
# target_link_libraries(test_graphdb ${OpenCV_LIBS} Probabilitieslib -lboost_unit_test_framework Maplib Comparatorlib VFLib)

# add_executable(graphdb Test/GraphDB/graphdb.cpp)
# target_link_libraries(graphdb ${OpenCV_LIBS} VFLib Probabilitieslib -lboost_unit_test_framework Maplib Comparatorlib -lboost_filesystem -lboost_system)

# add_executable(slam_images Test/GraphDB/slam_images.cpp)
# target_link_libraries(slam_images ${OpenCV_LIBS} VFLib Probabilitieslib Maplib Comparatorlib -lboost_filesystem -lboost_system)

add_executable(simulation_kth Test/GraphDB/simulation_kth.cpp)
target_link_libraries(simulation_kth ${OpenCV_LIBS} ${vodigrex_LIBRARIES} VFLib Comparatorlib -lboost_filesystem -lboost_system)

# add_executable(sketchvssketch Test/GraphDB/sketchVSsketch.cpp)
# target_link_libraries(sketchvssketch ${OpenCV_LIBS} ${vodigrex_LIBRARIES} VFLib Probabilitieslib Comparatorlib -lboost_filesystem -lboost_system)
# 
# add_executable(simulation_easy Test/GraphDB/simulation_easy.cpp)
# target_link_libraries(simulation_easy ${OpenCV_LIBS} VFLib Probabilitieslib Maplib Comparatorlib -lboost_filesystem -lboost_system)

# add_executable(simulation_corridor Test/GraphDB/simulation_corridor.cpp)
# target_link_libraries(simulation_corridor ${OpenCV_LIBS} VFLib Probabilitieslib Maplib Comparatorlib -lboost_filesystem -lboost_system)

# add_executable(simulation_intel Test/GraphDB/simulation_intel.cpp)
# target_link_libraries(simulation_intel ${OpenCV_LIBS} VFLib Probabilitieslib Maplib Comparatorlib -lboost_filesystem -lboost_system)

# add_executable(vfl_comp Test/GraphDB/VFLcomp/vfl_comp.cpp)
# target_link_libraries(vfl_comp ${OpenCV_LIBS}  VFLib Probabilitieslib -lboost_unit_test_framework Maplib Comparatorlib -lboost_filesystem -lboost_system)

add_executable(simulation_taurob Test/GraphDB/simulation_taurob.cpp)
target_link_libraries(simulation_taurob ${OpenCV_LIBS} ${vodigrex_LIBRARIES} VFLib Comparatorlib -lboost_filesystem -lboost_system)


# add_executable(scale_space Test/BaseTest/test_scale_space.cpp)
# target_link_libraries(scale_space ${OpenCV_LIBS} ${vodigrex_LIBRARIES})
# # 

# add_subdirectory(includes/boost/algorithm/sequence/)



# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)



add_subdirectory(includes)

add_subdirectory(Test/BaseTest)
add_subdirectory(Test/GraphDB)
add_subdirectory(Test/GraphDB/VFLcomp)
add_subdirectory(Test/SLAM)
add_subdirectory(Test/Simulation)

# 
# INSTALL(FILES sketchalgorithm-config.cmake DESTINATION lib/sketchAlgorithm)
# INSTALL(TARGETS Comparatorlib DESTINATION lib/sketchAlgorithm)
# INSTALL(TARGETS Probabilitieslib DESTINATION lib/sketchAlgorithm)
# INSTALL(TARGETS VFLib DESTINATION lib/sketchAlgorithm)
# 
# install(TARGETS simulation_kth RUNTIME DESTINATION bin)
# install(TARGETS simulation_taurob RUNTIME DESTINATION bin)


