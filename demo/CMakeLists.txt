cmake_minimum_required (VERSION 2.8)

project(TestIndem)

#set include
FILE(GLOB_RECURSE TEST_HEADER_FILES include/*.h *.h)
file(GLOB_RECURSE TEST_SOURCE_FILES *.cpp)
set(HEADER_DIR include)
find_path(GTEST_DIR include/gtest/gtest.h PATHS D:/git-ci/third_party)
include_directories(${TEST_HEADER_FILES} ${GTEST_INCLUDE_DIR})

#generate dll
add_executable(${PROJECT_NAME} ${TEST_SOURCE_FILES} ${TEST_HEADER_FILES})
#target_compile_options(TestIndem PUBLIC "/MT")

#set link libraries
if(WIN32)
target_link_libraries(${PROJECT_NAME}
    ${GTEST_LIBS} indem)
else(WIN32)
target_link_libraries(${PROJECT_NAME} gtest indem ${SLAM_LIB} tbb 
	cholmod
	boost_system boost_filesystem
	protobuf protobuf-lite nanomsg stdc++fs)
endif(WIN32)

install(TARGETS ${PROJECT_NAME}
RUNTIME DESTINATION ${TEST_BIN_DIR})