cmake_minimum_required(VERSION 3.8)
project(onrobot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# set message files
set(service_files
  "srv/IsConn.srv"
  "srv/IsGripped.srv"
  "srv/GetExtWidth.srv"
  "srv/GetIntWidth.srv"
  "srv/GripExternal.srv"
  "srv/GripInternal.srv"
  "srv/GetFingerValues.srv"
  "srv/GetMaxMinWidth.srv"
)

# generate interfaces

rosidl_generate_interfaces(${PROJECT_NAME}
  ${service_files}
  DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
