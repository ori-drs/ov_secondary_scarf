cmake_minimum_required(VERSION 3.8)
project(ov_secondary_loop_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")

find_package(rclcpp REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3)

find_package(cv_bridge REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ov_secondary_camera_models REQUIRED)
find_package(rosbag2_cpp REQUIRED)


include_directories(include ${CERES_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

add_executable(loop_fusion_node
    src/loop_fusion_node.cpp
    src/loop_fusion_io.cpp
    src/pose_graph.cpp
    src/keyframe.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp
    )

target_link_libraries(loop_fusion_node ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${CERES_LIBRARIES}) 


install(
  DIRECTORY launch scripts config data
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS loop_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)



ament_target_dependencies(loop_fusion_node
  rclcpp
  builtin_interfaces
  nav_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  ament_index_cpp
  ov_secondary_camera_models
  rosbag2_cpp
)
ament_package()
