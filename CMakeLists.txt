#cmake_minimum_required(VERSION 4.0)
cmake_minimum_required(VERSION 3.22)
project(untitled2)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 1. 设置 CMAKE_PREFIX_PATH 在 find_package 之前
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")

# 2. 查找 ament_cmake
find_package(ament_cmake REQUIRED)

#set(rosidl_typesupport_c_DIR "/opt/ros/humble/share/rosidl_typesupport_c/cmake")
# 3. 查找 ROS 2 包
find_package(rosidl_typesupport_c REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)

# 显式添加 ROS 2 头文件路径
include_directories(
        ${geometry_msgs_INCLUDE_DIRS}
        ${tf2_geometry_msgs_INCLUDE_DIRS}
        /opt/ros/humble/include
)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# 迈德威视SDK配置
set(MVSDK_ROOT "/home/liufy/linuxSDK_V2.1.0.37")
include_directories("${MVSDK_ROOT}/include")  # 头文件路径

# 根据系统架构选择库路径
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    link_directories("${MVSDK_ROOT}/lib/x64")  # 64位系统
else()
    link_directories("${MVSDK_ROOT}/lib/x86")  # 32位系统
endif()



# 确保使用正确的Python解释器
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}")


add_executable(untitled2
        #       main.cpp
        /home/liufy/CLionProjects/untitled2/noto/armor.cpp
        /home/liufy/CLionProjects/untitled2/noto/armor.h
        /home/liufy/CLionProjects/untitled2/noto/armorBox.cpp
        /home/liufy/CLionProjects/untitled2/noto/armorNum.cpp
        /home/liufy/CLionProjects/untitled2/noto/EKF.cpp
        /home/liufy/CLionProjects/untitled2/noto/EKF.h
        /home/liufy/CLionProjects/untitled2/noto/kalman.cpp
        /home/liufy/CLionProjects/untitled2/noto/LightBar.cpp
        /home/liufy/CLionProjects/untitled2/noto/matchArmor.cpp
        /home/liufy/CLionProjects/untitled2/noto/pnp.cpp
        /home/liufy/CLionProjects/untitled2/noto/tracker.cpp
        /home/liufy/CLionProjects/untitled2/noto/tracker.h
)
ament_target_dependencies(untitled2
        rclcpp
        geometry_msgs
        tf2_geometry_msgs
        builtin_interfaces
        tf2_ros
        angles
        rosidl_typesupport_c
)

# 添加包含路径
target_include_directories(untitled2 PUBLIC
#        ${rclcpp_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${tf2_geometry_msgs_INCLUDE_DIRS}
#        ${builtin_interfaces_INCLUDE_DIRS}
        ${tf2_ros_INCLUDE_DIRS}
        ${angles_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/noto>  # 自定义头文件路径
)

# 链接库
target_link_libraries(untitled2
#        ${rclcpp_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${tf2_geometry_msgs_LIBRARIES}
        ${OpenCV_LIBS}
        MVSDK
)


ament_package()
