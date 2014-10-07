cmake_minimum_required(VERSION 2.4.6)

include($ENV{LIBPYARR_ROOT}/common.cmake)
#include($ENV{LIB_DM_CPP}/common.cmake)

add_library(motion_model
  motion_model.cc)

add_library(observation_model
  observation_model.cc)

add_library(map_object
  map_object.cc)

set_target_properties(motion_model PROPERTIES SUFFIX ".so")
set_target_properties(observation_model PROPERTIES SUFFIX ".so")
set_target_properties(map_object PROPERTIES SUFFIX ".so")

#add_library(chunk_in_prog_candidate
#  chunk_in_prog_candidate.cpp)
#set_target_properties(chunk_in_prog_candidate PROPERTIES SUFFIX ".so")

message("python:" ${PYTHON_LIBRARIES})

set(CMAKE_CXX_FLAGS "-fopenmp")
#set(CMAKE_BUILD_TYPE Debug)

target_link_libraries(motion_model ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} boost_common)
target_link_libraries(map_object ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} boost_common)
target_link_libraries(observation_model ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} boost_common motion_model map_object)
