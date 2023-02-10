cmake_minimum_required(VERSION 3.16)
project(dvl)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rtac_asio REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  src/dvl_parser_node.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(driver src/driver.cpp)



target_link_libraries(driver PUBLIC
  ${ament_LIBRARIES}
)
target_include_directories(driver PUBLIC
  ${serial_INCLUDE_DIRS}
)
target_link_libraries(driver 
  ${serial_LIBRARIES}
  rtac_asio
)
include_directories(include)
target_compile_features(driver PRIVATE cxx_std_17)

ament_target_dependencies(driver rclcpp)

install(TARGETS
  driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  src
  DESTINATION share/${PROJECT_NAME}/
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
