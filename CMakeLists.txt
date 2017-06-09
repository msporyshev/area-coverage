cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

find_package(CGAL REQUIRED COMPONENTS)

include(${CGAL_USE_FILE})

# # link_directories(
# #   ${CGAL_LIBRARY_DIRS}
# #   )

# # include_directories(
# #   ${CGAL_INCLUDE_DIRS}
# #   )

# # message("${CGAL_LIBRARIES}")


# # target_link_libraries(main ${CGAL_LIBRARIES} )

# include( CGAL_CreateSingleSourceCGALProgram )
# create_single_source_cgal_program( "main.cpp")

set(CMAKE_CXX_STANDARD 14)

find_package(Boost REQUIRED COMPONENTS system)


link_directories(
  ${CGAL_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
  )

include_directories(
  ${CGAL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )


add_executable(main source.cpp)
target_link_libraries(main ${CGAL_LIBRARIES} ${Boost_LIBRARIES})

