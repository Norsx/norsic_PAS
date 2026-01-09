cmake_minimum_required(VERSION 3.16)
project(student_assignment_02 LANGUAGES NONE)

# Za Python paket s ROS 2 koristi ament_python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Instaliraj Python package - ovo pronalazi student_assignment_02 direktorij
ament_python_install_package(${PROJECT_NAME})

# Instaliraj launch datoteke
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Instaliraj config datoteke
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Instaliraj world datoteke
install(DIRECTORY
  world
  DESTINATION share/${PROJECT_NAME}
)

# Instaliraj mapped_maps
install(DIRECTORY
  mapped_maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
