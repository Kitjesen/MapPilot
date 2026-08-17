find_package(Boost REQUIRED COMPONENTS filesystem)
list(APPEND geometric_shapes_LIBRARIES ${Boost_LIBRARIES})