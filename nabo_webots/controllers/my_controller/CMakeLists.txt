#[[=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP with MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================]]

cmake_minimum_required(VERSION 3.0)
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
message("Project name:  " ${PROJECT})
project(${PROJECT})

if(MSVC)
	add_compile_options(/W3)
	add_compile_options(/wd4819)
	add_compile_options(/wd4244)
	# add webots wrappers for MSVC
	set(CPP_WRAPPER_SOURCES_DIR "D:/Webots/src/controller/cpp/")
	file(GLOB_RECURSE WRAPPER_SOURCES ${CPP_WRAPPER_SOURCES_DIR}*.cpp)
endif()

# Get C or C++ sources in the current directory (only).
set(SOURCES my_controller.cpp)
# add definitions
add_definitions("-D_USE_MATH_DEFINES")

# set include dirs
if(MSVC)
	set(webotsDir $ENV{WEBOTS_HOME})
elseif(UNIX)
	set(webotsDir "/usr/local/webots")
endif()

set(WEBOTS_INC_DIR ${webotsDir}/include/controller/c ${webotsDir}/include/controller/cpp)
include_directories(${WEBOTS_INC_DIR})

# Link with the Webots controller library.
link_directories(${webotsDir}/lib/controller)
if(MSVC)
	set (WEBOTS_C_LIBRARIES Controller.dll)
	add_library(webot_cpp_wrapper ${WRAPPER_SOURCES})
	target_link_libraries(webot_cpp_wrapper ${WEBOTS_C_LIBRARIES})
endif()

if(MSVC)
	set (WEBOTS_LIBS Controller.dll webot_cpp_wrapper)
else()
	set (WEBOTS_LIBS Controller CppController)
endif()


link_directories(.)
# Setup the target executable.
add_executable(${PROJECT} ${SOURCES})
target_link_libraries(${PROJECT} ${WEBOTS_LIBS} nabo)


# Copy the target executable at the right location.
if(MSVC)
	add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
		copy ${CMAKE_BINARY_DIR}/Release/${PROJECT}.exe ${CMAKE_SOURCE_DIR}
	)
else()
	add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
		copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
	)
endif()
