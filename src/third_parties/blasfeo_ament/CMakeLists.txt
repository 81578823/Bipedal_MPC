cmake_minimum_required(VERSION 3.8)
project(blasfeo_ament)

find_package(ament_cmake REQUIRED)

include(FetchContent)

# Define directories
set(BLASFEO_DEVEL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE STRING "Blasfeo install path")
set(BLASFEO_INCLUDE_DIR ${BLASFEO_DEVEL_PREFIX}/include)
set(BLASFEO_LIB_DIR ${BLASFEO_DEVEL_PREFIX}/lib)
set(BLASFEO_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
set(BLASFEO_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

# Create directories if they do not exist
file(MAKE_DIRECTORY ${BLASFEO_INCLUDE_DIR})
file(MAKE_DIRECTORY ${BLASFEO_LIB_DIR})
file(MAKE_DIRECTORY ${BLASFEO_DOWNLOAD_DIR})
file(MAKE_DIRECTORY ${BLASFEO_BUILD_DIR})

# BLASFEO Settings
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(TARGET X64_AUTOMATIC CACHE STRING "Target architecture" FORCE)
set(BLASFEO_EXAMPLES OFF CACHE BOOL "Examples enabled")

set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -DLA_${LA}")
set(BLASFEO_REF_API ON)

if(${BLASFEO_REF_API})
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DBLASFEO_REF_API")
endif()

# Download & build source
FetchContent_Declare(blasfeoDownload
	GIT_REPOSITORY https://github.com/giaf/blasfeo.git
	GIT_TAG master
	UPDATE_COMMAND ""
	SOURCE_DIR ${BLASFEO_DOWNLOAD_DIR}
	BINARY_DIR ${BLASFEO_BUILD_DIR}
	BUILD_COMMAND "$(MAKE) shared_library"
	INSTALL_COMMAND "$(MAKE) install_shared"
)
FetchContent_MakeAvailable(blasfeoDownload)

# Copy header to where catkin expects them
file(GLOB_RECURSE HEADERS "${BLASFEO_DOWNLOAD_DIR}/include/*.h")

foreach(HEADER_FILE ${HEADERS})
	message(STATUS "FOUND HEADER: " ${HEADER_FILE})
	file(COPY ${HEADER_FILE} DESTINATION ${BLASFEO_INCLUDE_DIR})
endforeach()

# Install the library where catkin expects them
set_target_properties(blasfeo PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${BLASFEO_LIB_DIR})

ament_export_libraries(blasfeo)
ament_package(CONFIG_EXTRAS cmake/blasfeo-extras.cmake.in)