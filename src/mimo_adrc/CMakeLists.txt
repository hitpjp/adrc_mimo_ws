cmake_minimum_required(VERSION 3.8)
project(mimo_adrc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -mavx2 -mfma)
endif()

# 1. 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 2. 包含头文件目录
include_directories(include)

# 3. 编译库 (ADRC核心算法)
add_library(adrc_lib src/ADRC.cpp src/MimoADRC.cpp)
target_include_directories(adrc_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 4. 编译节点 (仿真主程序)
add_executable(mimo_node src/mimo_node.cpp)
target_link_libraries(mimo_node adrc_lib)

# 5. 链接依赖项 (这里一定要全)
ament_target_dependencies(mimo_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 6. 安装规则
install(TARGETS adrc_lib mimo_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(adrc_lib)
ament_package()