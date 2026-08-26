cmake_minimum_required(VERSION 3.16)
project(octo_planner3d LANGUAGES C CXX)

# 导出 compile_commands.json，供 IDE (VSCode/clangd) 解析头文件路径
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 解决 Ubuntu libvtk9-dev 打包 bug：VTK-targets.cmake 在定义 VTK::mpi 时引用了
# 未定义的 MPI::MPI_C（vtk-config.cmake 先 include targets，后才 find MPI，顺序反了）。
# 提前 find_package(MPI) 创建真实 MPI::MPI_C 目标，供 PCL→VTK 配置阶段校验通过。
# 该 bug 在 arm64 与 amd64 上均存在，故两种架构原生编译都需要这一步。
find_package(MPI REQUIRED)

find_package(PCL REQUIRED COMPONENTS io common)
find_package(ament_cmake QUIET)

# OctoMap 通过系统包提供：apt install liboctomap-dev
# find_package(OCTOMAP) 会导入 octomap / octomath 两个 IMPORTED 目标，
# 目标名与原先手写的保持一致，下游子目录无需改动。
find_package(OCTOMAP REQUIRED)

add_subdirectory(octomap)
add_subdirectory(planner)

add_executable(globalPlan main.cpp)
target_link_libraries(globalPlan PRIVATE GlobalPlanner Octomap)

if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)

  add_executable(octo_planner_rviz_node src/octo_planner_rviz_node.cpp)
  target_link_libraries(octo_planner_rviz_node GlobalPlanner Octomap)
  set_target_properties(octo_planner_rviz_node PROPERTIES
    INSTALL_RPATH "$ORIGIN"
  )
  target_compile_definitions(
    octo_planner_rviz_node
    PRIVATE OCTO_PLANNER3D_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_target_dependencies(
    octo_planner_rviz_node
    rclcpp
    geometry_msgs
    nav_msgs
    std_msgs
    visualization_msgs
    tf2_ros
  )

  install(TARGETS
    globalPlan
    octo_planner_rviz_node
    DESTINATION lib/${PROJECT_NAME}
  )

  # 注：不再随包安装 liboctomap/liboctomath，改由目标机通过系统包
  # (apt install liboctomap1.9 运行时库) 提供。

  install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/octo_planner.rviz"
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()
endif()
