add_subdirectory(MapComparator)
add_subdirectory(ProbabilisticMethod)
add_subdirectory(TopologicalMap)
add_subdirectory(VFLConversion)


FILE(GLOB files_vflib "${CMAKE_CURRENT_SOURCE_DIR}/VFLib/include/*.h")
INSTALL(FILES ${files_vflib} DESTINATION include/sketchAlgorithm/VFLib)