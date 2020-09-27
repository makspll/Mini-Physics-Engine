cmake_minimum_required(VERSION 3.16)


set(MAIN_TARGET "MiniPhysicsEngine")

project(${MAIN_TARGET}
			  DESCRIPTION "A physics engine based on Ian Millington's book"
			  LANGUAGES CXX)


set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(SRC_PATH "${PROJECT_SOURCE_DIR}/src")

include_directories(${SRC_PATH})
file(GLOB SRCFILES ${SRC_PATH}/*.cpp)



add_executable(${MAIN_TARGET}
		${SRCFILES})
		
find_package(Armadillo REQUIRED)

if(ARMADILLO_FOUND)
	target_include_directories(${MAIN_TARGET} PRIVATE ${ARMADILLO_INCLUDE_DIRS})
	target_link_libraries(${MAIN_TARGET} ${ARMADILLO_LIBRARIES})
else()
	message(FATAL_ERROR "Could not find the armadillo library")
endif()


find_package(allegro 5.1 COMPONENTS main image primative color)

if(ALLEGRO_FOUND)
	include_directories(${ALLEGRO_INCLUDE_DIRS})
	target_link_libraries(${MAIN_TARGET} ${ALLEGRO_LIBRARIES})
else()
	message(FATAL_ERROR "Could not find the allegro library")
endif()



