cmake_minimum_required(VERSION 3.16)
project(dummy_vc_moveit_config)

find_package(ament_cmake REQUIRED)

# MoveIt 的配置包只有 launch / config / urdf 这类数据，没有要编译的东西。
# 装到 share/ 下即可。
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
