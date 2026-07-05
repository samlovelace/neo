cmake_minimum_required(VERSION 3.8)
project(neo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ptera_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(plog REQUIRED)

set(SOURCES
	src/PerceptionInterface.cpp
	src/main.cpp
	src/VehicleInterface.cpp
	src/DataLogger.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
	${CMAKE_CURRENT_LIST_DIR}/include
	/opt/ros/humble/include
)

target_link_libraries(${PROJECT_NAME} plog::plog)

ament_target_dependencies(${PROJECT_NAME}
	rclcpp
	ament_index_cpp
	ptera_msgs
	behaviortree_cpp
	Eigen3
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY trees/ DESTINATION share/${PROJECT_NAME}/trees)
ament_package()
