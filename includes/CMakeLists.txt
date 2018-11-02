add_subdirectory(MapComparator)
add_subdirectory(ProbabilisticMethod)
add_subdirectory(TopologicalMap)
add_subdirectory(VFLConversion)
add_subdirectory(LaplacianGraphMatching)


find_package(RSI)

if ( RSI_FOUND )
  
  include_directories(${RSI_INCLUDE_DIRS})
  add_subdirectory(Conversion)
  
endif()


FILE(GLOB files_vflib "${CMAKE_CURRENT_SOURCE_DIR}/VFLib/include/*.h")
INSTALL(FILES ${files_vflib} DESTINATION include/sketchAlgorithm/VFLib)
