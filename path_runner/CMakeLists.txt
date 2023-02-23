cmake_minimum_required(VERSION 3.22)
project(path_runner)


# find dependencies
find_package(ament_cmake REQUIRED)

install(
	DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
