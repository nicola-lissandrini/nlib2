cmake_minimum_required(VERSION 3.8)
project(nlib2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories (
	include
)

install (
	DIRECTORY include/
	DESTINATION include
)

install(PROGRAMS
  scripts/ros_configuration.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()
