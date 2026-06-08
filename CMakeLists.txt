cmake_minimum_required(VERSION 3.21)
project(sfml_example LANGUAGES CXX)

include(FetchContent)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SFML_STATIC_LIBRARIES TRUE)

# Point CMake to your SFML 3 package config.
# This should contain SFMLConfig.cmake, usually under lib/cmake/SFML.
if(NOT SFML_DIR OR SFML_DIR STREQUAL "SFML_DIR-NOTFOUND")
	set(SFML_DIR "C:/SFML-custom/lib/cmake/SFML" CACHE PATH "Path to SFMLConfig.cmake" FORCE)
endif()

# Some prebuilt MinGW SFML packages ship debug DLLs that mismatch newer GCC runtimes.
# This maps Debug to Release SFML binaries so the app runs reliably.
if(MINGW)
	set(CMAKE_MAP_IMPORTED_CONFIG_DEBUG Release)
endif()

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# Try to find GLUT on the system first
find_package(GLUT QUIET)

# If found, use it; otherwise skip (GLUT is optional - modern OpenGL doesn't require it)
if(GLUT_FOUND)
	message(STATUS "Found system GLUT: ${GLUT_LIBRARIES}")
else()
	message(STATUS "GLUT not found on system - GLUT/GLU features will not be available")
	# To add GLUT support: install freeglut-devel or build freeglut and set up CMAKE_PREFIX_PATH
	set(GLUT_INCLUDE_DIR "")
	set(GLUT_LIBRARIES "")
endif()

find_package(OpenGL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(SFML CONFIG REQUIRED COMPONENTS Graphics Window System)

# Auto-pick all project source files, e.g. src/MVec.cpp
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_executable(sfml_example ${APP_SOURCES})
if(GLUT_INCLUDE_DIR)
	target_include_directories(sfml_example PRIVATE ${GLUT_INCLUDE_DIR})
endif()
target_link_libraries(sfml_example PRIVATE SFML::Graphics SFML::Window SFML::System OpenGL::GL)
if(GLUT_LIBRARIES)
	target_link_libraries(sfml_example PRIVATE ${GLUT_LIBRARIES})
endif()
if(WIN32)
	target_link_libraries(sfml_example PRIVATE opengl32 glu32)
endif()

if(WIN32)
	# Copy SFML and compiler runtime DLLs next to the exe for local runs.
	add_custom_command(TARGET sfml_example POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy -t $<TARGET_FILE_DIR:sfml_example> $<TARGET_RUNTIME_DLLS:sfml_example>
		COMMAND_EXPAND_LISTS
	)
endif()
