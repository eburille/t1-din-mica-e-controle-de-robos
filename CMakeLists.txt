cmake_minimum_required(VERSION 3.20)
project(t1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(trajectory_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(jnt_cart src/jnt_cart.cpp)
target_compile_features(jnt_cart PUBLIC c_std_17 cxx_std_20)  # Require C17 and C++20
target_include_directories(jnt_cart PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(
    jnt_cart
    ${rclcpp_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${kdl_parser_TARGETS}
    ${orocos_kdl_TARGETS}  
    ${trajectory_msgs_TARGETS}
  )

add_executable(gerador_trajetoria src/gerador_trajetoria.cpp)
target_compile_features(gerador_trajetoria PUBLIC c_std_17 cxx_std_20)  # Require C17 and C++20
target_link_libraries(
    gerador_trajetoria
    ${rclcpp_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${kdl_parser_TARGETS}
    ${orocos_kdl_TARGETS}  
    ${trajectory_msgs_TARGETS}
  )

add_executable(twist_publisher src/twist_publisher.cpp)
target_compile_features(twist_publisher PUBLIC c_std_17 cxx_std_20)  # Require C17 and C++20
target_link_libraries(
  twist_publisher
  ${rclcpp_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${tf2_kdl_TARGETS}
  ${urdf_TARGETS}
  ${kdl_parser_TARGETS}
  ${orocos_kdl_TARGETS}
)

add_executable(controlador src/controlador.cpp)
target_compile_features(controlador PUBLIC c_std_17 cxx_std_20)  # Require C17 and C++20
target_link_libraries(
  controlador
  ${rclcpp_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${kdl_parser_TARGETS}
  ${orocos_kdl_TARGETS}
  ${tf2_kdl_TARGETS}
  ${urdf_TARGETS}
  ${trajectory_msgs_TARGETS}
)

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS jnt_cart
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS gerador_trajetoria
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS twist_publisher
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS controlador
  DESTINATION lib/${PROJECT_NAME})

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