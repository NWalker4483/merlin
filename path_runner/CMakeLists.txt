project(path_runner)

cmake_minimum_required(VERSION 3.22)
install(
	DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME}
)