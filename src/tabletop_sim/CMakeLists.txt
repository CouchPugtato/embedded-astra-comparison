cmake_minimum_required(VERSION 3.16)
project(tabletop_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(PROGRAMS scripts/scene_setup.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scene_setup
)

install(PROGRAMS scripts/run_attempt.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME run_attempt
)

install(PROGRAMS scripts/prepare_scene.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME prepare_scene
)

install(PROGRAMS scripts/replay_policy.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME replay_policy
)

install(FILES scripts/grading.py scripts/image_file.py scripts/reset_environment.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
