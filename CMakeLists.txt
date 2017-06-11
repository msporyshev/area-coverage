cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

find_package(CGAL REQUIRED COMPONENTS)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)

include(${CGAL_USE_FILE})


find_package(Boost REQUIRED COMPONENTS system)


link_directories(
  ${CGAL_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
  )

include_directories(
  ${CGAL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )


add_executable(main main.cpp geom.cpp planner.cpp graph.cpp)
target_link_libraries(main ${CGAL_LIBRARIES} ${Boost_LIBRARIES})

