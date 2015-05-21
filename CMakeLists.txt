# project
cmake_minimum_required (VERSION 2.6)
project (libviso2)

set(VISO2 viso2)
# directories
set (LIBVISO2_SRC_DIR src)

# include directory
include_directories("${LIBVISO2_SRC_DIR}")

# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

# sources
FILE(GLOB LIBVISO2_SRC_FILES "src/*.cpp")

# make release version
set(CMAKE_BUILD_TYPE Release)

# demo program
add_executable(${VISO2} ${LIBVISO2_SRC_FILES})

# find opencv package
list(APPEND CMAKE_MODULE_PATH "/usr/local/Cellar")

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
   include_directories(${OpenCV_INCLUDE_DIRS})
   target_link_libraries(${VISO2} ${OpenCV_LIBRARIES})
else (OpenCV_FOUND)
	message(STATUS "OPENCV PACKAGE NOT FOUND")
endif (OpenCV_FOUND)

find_package(PCL REQUIRED)
if (PCL_FOUND)
   include_directories(${PCL_INCLUDE_DIRS})

   target_link_libraries(${VISO2} ${PCL_COMMON_LIBRARIES})
   target_link_libraries(${VISO2} ${PCL_IO_LIBRARIES})
   target_link_libraries(${VISO2} ${PCL_KDTREE_LIBRARIES})
   target_link_libraries(${VISO2} ${PCL_FILTERS_LIBRARIES})
   target_link_libraries(${VISO2} ${PCL_FEATURES_LIBRARIES})
   target_link_libraries(${VISO2} ${PCL_VISUALIZATION_LIBRARIES})
else (PCL_FOUND)
    message(STATUS "PCL not Found")
endif (PCL_FOUND)

# link tesseract libraries
set(LIBS_DIR "/usr/local/lib")
set(TESSERACT_INCLUDE_DIR "/usr/local/include/tesseract")
include_directories(${TESSERACT_INCLUDE_DIR})
link_directories(${LIBS_DIR})
target_link_libraries(${VISO2} tesseract.3)
