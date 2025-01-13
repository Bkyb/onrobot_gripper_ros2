cmake_minimum_required(VERSION 3.8)
project(onrobot_2fg7)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(onrobot_msgs REQUIRED)

include_directories(/usr/include/xmlrpc-c)
link_directories(/usr/lib/x86_64-linux-gnu)

set(ROS2_PACKAGES
    rclcpp
    std_msgs
    onrobot_msgs
)

include_directories(${PROJECT_SOURCE_DIR}/include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(onrobot_server_node
    src/onrobot_server_node.cpp
    src/onrobot_server.cpp
    src/twofg_xmlrpc.cpp
)

target_link_libraries(onrobot_server_node
  xmlrpc++
  xmlrpc_client++
  xmlrpc_cpp
  xmlrpc_packetsocket
  xmlrpc_server++
  xmlrpc_server_abyss++
)

ament_target_dependencies(onrobot_server_node ${ROS2_PACKAGES})
install(TARGETS onrobot_server_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
