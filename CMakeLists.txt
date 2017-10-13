cmake_minimum_required(VERSION 2.8)
project(oc_ndt)

find_package(catkin REQUIRED COMPONENTS
 sensor_msgs tf
message_generation geometry_msgs std_msgs
cmake_modules
  roscpp
  sensor_msgs
  nodelet
  pcl_ros
)
find_package(octomap REQUIRED)
# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io )

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

catkin_package(
  DEPENDS sensor_msgs tf  message_runtime geometry_msgs std_msgs)
#${PROJECT_NAME}

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# octomap
#FIND_PACKAGE( octomap REQUIRED )
#INCLUDE_DIRECTORIES( ${OCTOMAP_INCLUDE_DIRS} )

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
link_libraries( ${catkin_LIBRARIES})
#${PROJECT_NAME}

include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries( ${OCTOMAP_LIBRARIES})
#${PROJECT_NAME}

include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(uniformDivision
     include/Stopwatch.h
      include/Vec2.h
     include/Vec3.h
     include/2Dmap.h
 src/uniformDivision.cpp)
target_link_libraries(uniformDivision
${PCL_LIBRARIES}
${OpenCV_LIBS} )

#add_executable(octreeDivision
#     include/Stopwatch.h
#      include/Vec2.h
#     include/Vec3.h
# src/octreeDivision.cpp)
#target_link_libraries(octreeDivision
#${PCL_LIBRARIES}
#${OpenCV_LIBS} )

#add_executable(pclCountNormal
# src/pclCountNormal.cpp)
#target_link_libraries(pclCountNormal
#${PCL_LIBRARIES}
#${OpenCV_LIBS} )

#add_executable(test123
# src/test.cpp)
#target_link_libraries(test123
#${PCL_LIBRARIES}
#${OpenCV_LIBS} )

#install(TARGETS ${PROJECT_NAME}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(DIRECTORY include/octomap_ros/
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/octomap_ros/
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE)



