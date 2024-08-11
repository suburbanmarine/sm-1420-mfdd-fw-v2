cmake_minimum_required(VERSION 3.10)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

project(sm-1420-mfdd
	VERSION 0.1.0
	DESCRIPTION "Smart PDU"
	LANGUAGES C CXX
)


add_executable(sm-1420-mfdd
	main.cpp
)