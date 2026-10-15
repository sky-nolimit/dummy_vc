cmake_minimum_required(VERSION 3.16)
project(dummy_vc_moveit_config)

find_package(ament_cmake REQUIRED)

# MoveIt 的配置包只有 launch / config / urdf 这类数据，没有要编译的东西。
# 装到 share/ 下即可。
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 单独把 .setup_assistant 装到 share 根目录 —— 不能用上面那条。
#
# MoveItConfigsBuilder 只在 <share>/.setup_assistant 找这个文件，也就是包根，
# 不在 <share>/config/ 下找。上面那条 install(DIRECTORY config ...) 只会把它
# 放到 <share>/config/.setup_assistant，builder 看不见，于是回退成
# config/dummy2.urdf（不存在）→ robot_description 为空。
#
# 症状是一串谁都不指向根因的报错：
#   Cannot infer URDF from `<share>` -- using config/dummy2.urdf
#   Cannot infer SRDF from `<share>` -- using config/dummy2.srdf
#   "File <share>/config/dummy2.urdf doesn't exist"
#   robot_description parameter must not be empty (robot_state_publisher)
#
# 源文件仍然放在 config/ 下（和 Setup Assistant 自己的习惯一致），
# 只在这里换个目的地。
install(
  FILES config/.setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
