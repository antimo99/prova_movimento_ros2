cmake_minimum_required(VERSION 3.8)
project(prova_movimento_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(rclcpp_action)
find_package(sensor_msgs)
find_package(control_msgs)
find_package(chrono)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(lettura_singola src/lettura_singola.cpp)
ament_target_dependencies(lettura_singola
      "rclcpp"  "sensor_msgs")

add_executable(lettura_continua src/lettura_continua.cpp)
ament_target_dependencies(lettura_continua
      "rclcpp" "sensor_msgs")

add_executable(prova3 src/prova3.cpp)
ament_target_dependencies(prova3
      "rclcpp" "rclcpp_action" "sensor_msgs" "control_msgs")

add_executable(prova4 src/prova4.cpp)
ament_target_dependencies(prova4
       "rclcpp" "rclcpp_action" "sensor_msgs" "control_msgs")
      
install(TARGETS 
        lettura_singola lettura_continua prova3 prova4
       DESTINATION lib/${PROJECT_NAME}
       )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
