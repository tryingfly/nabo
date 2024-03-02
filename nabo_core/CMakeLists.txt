#[[=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================]]

cmake_minimum_required(VERSION 2.8.12)
project(manager)
set(CMAKE_BUILD_TYPE "Release")
if(UNIX)
	add_definitions(-std=c++11)
	add_compile_options(-fPIC)
else()
	# add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
	add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
	set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS on)
	set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

set(dirRbdl "third_party/rbdl")
set(dirEiQP "third_party/eiquadprog")
set(dirUtil "utility")
set(incEIGEN "third_party/eigen3")

set(dirNabo "src/robot" "src/balance" "src/plan" "src/common"  "src/manager" "src/estimate")
set(allInc ${dirUtil} ${dirRbdl} ${dirEiQP} ${incEIGEN} ${dirNabo})
set(allLib ${dirRbdl} ${dirEiQP})
include_directories(${allInc})
link_directories(${allLib})

# ================================
file(GLOB_RECURSE utilFiles ${dirUtil}/*.cpp)
add_library(util STATIC ${utilFiles})
# -----------
file(GLOB_RECURSE srcFiles "src/*.cpp")
add_library(nabo SHARED ${srcFiles})
if(MSVC)
	target_link_libraries(nabo util rbdl EiQP)
elseif(UNIX)
	target_link_libraries(nabo util rbdl EiQP pthread)
endif()
# -----------
aux_source_directory(test testFiles)
add_executable(main ${testFiles})
target_link_libraries(main nabo)
# ===============================
add_custom_command(TARGET nabo POST_BUILD COMMAND ${CMAKE_COMMAND} -E
	make_directory ${CMAKE_SOURCE_DIR}/nabo_output
)
if(MSVC)
	add_custom_command(TARGET nabo POST_BUILD COMMAND ${CMAKE_COMMAND} -E
		copy ${CMAKE_BINARY_DIR}/Release/nabo.dll
			${CMAKE_BINARY_DIR}/Release/nabo.lib
			${CMAKE_SOURCE_DIR}/src/manager/nabo.h
			${CMAKE_SOURCE_DIR}/src/manager/nabo_config.h
			${CMAKE_SOURCE_DIR}/nabo_output
	)
elseif(UNIX)
	add_custom_command(TARGET nabo POST_BUILD COMMAND ${CMAKE_COMMAND} -E
		copy ${CMAKE_BINARY_DIR}/libnabo.so
			${CMAKE_SOURCE_DIR}/src/manager/nabo.h
			${CMAKE_SOURCE_DIR}/src/manager/nabo_config.h
			${CMAKE_SOURCE_DIR}/nabo_output
	)
endif()