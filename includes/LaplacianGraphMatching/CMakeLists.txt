add_library(GraphLaplacianLib
        GraphLaplacian.cpp
        GraphMatcherNeighborLaplacian.cpp
        HypotheseLaplacian.cpp
        GraphMatcherBaseLaplacian.cpp
        )
target_link_libraries(GraphLaplacianLib ${RSI_LIBRARIES})