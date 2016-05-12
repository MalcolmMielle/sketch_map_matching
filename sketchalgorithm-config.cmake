
get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(sketchAlgorithm_INCLUDE_DIRS "${SELF_DIR}/../../include/sketchAlgorithm" ABSOLUTE)
set(sketchAlgorithm_LIBRARIES "${SELF_DIR}/libComparatorlib.a;${SELF_DIR}/libProbabilitieslib.a;${SELF_DIR}/libVFLib.a")
