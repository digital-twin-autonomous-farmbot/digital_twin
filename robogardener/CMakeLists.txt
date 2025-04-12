cmake_minimum_required(VERSION 3.5)
project(robogardener)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python nodes
install(PROGRAMS
  nodes/state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Make Python scripts executable
file(GLOB PYTHON_SCRIPTS
  "nodes/*.py"
)
foreach(py_script IN LISTS PYTHON_SCRIPTS)
  execute_process(
    COMMAND chmod +x ${py_script}
  )
endforeach()

install(
  DIRECTORY config description launch nodes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()