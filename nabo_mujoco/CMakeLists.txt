#[[=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP with MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================]]

cmake_minimum_required(VERSION 3.0)
project(test)
set(CMAKE_BUILD_TYPE "Release")
if(MSVC)
	# add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
	add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
elseif(UNIX)
	add_definitions(-std=c++11)
endif()

set(allDir "mujoco" "glfw" "nabo")
include_directories(${allDir})
link_directories(${allDir})

aux_source_directory(. srcFiles)
add_executable(main ${srcFiles})
target_link_libraries(main mujoco glfw3 nabo)

if(MSVC)
	add_custom_command(TARGET main POST_BUILD COMMAND ${CMAKE_COMMAND} -E
		copy ${CMAKE_SOURCE_DIR}/mujoco/mujoco.dll
			${CMAKE_SOURCE_DIR}/nabo/nabo.dll
			${CMAKE_BINARY_DIR}/Release
	)
elseif(UNIX)
	add_custom_command(TARGET main POST_BUILD COMMAND ${CMAKE_COMMAND} -E
		copy ${CMAKE_SOURCE_DIR}/mujoco/libmujoco.so
			${CMAKE_SOURCE_DIR}/glfw/libglfw3.so
			${CMAKE_SOURCE_DIR}/nabo/libnabo.so
			${CMAKE_BINARY_DIR}
	)
endif()
