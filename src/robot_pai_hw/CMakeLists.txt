cmake_minimum_required(VERSION 3.10)
project(robot_pai_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17") 
SET(CMAKE_BUILD_TYPE "Release")
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        std_msgs
        tf
        serial
        controller_manager
        robot_state_publisher
        )

set(pthread_LIBRARIES pthread)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        serial
        std_msgs
        tf
        robot_state_publisher
        controller_manager
)

find_library(SERIALPORT_LIBRARY serialport)

###########
## Build ##
###########

set(serial_SRCS 
    src/serial/impl/unix.cc
    src/serial/impl/list_ports/list_ports_linux.cc
    src/serial/serial.cc  
)

set(hardware_SRCS
    src/hardware/motor/motor.cc
    src/hardware/lively_serial.cc
    src/RobotPaiHW.cpp
    src/RobotPaiHWLoop.cpp
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
            ${serial_SRCS}
            ${hardware_SRCS}
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${SERIALPORT_LIBRARY}
        # rt 
        # pthread
        )

# ros node、
add_executable(test_motor_run src/test/test_motor_run.cpp)
target_link_libraries(test_motor_run ${PROJECT_NAME} ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})# ${SERIAL_LIB}

add_executable(test_reset_zero src/test/test_reset_zero.cpp)
target_link_libraries(test_reset_zero ${PROJECT_NAME} ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})# ${SERIAL_LIB}

add_executable(${PROJECT_NAME}_node src/RobotPaiHW_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${SERIALPORT_LIBRARY})# ${SERIAL_LIB}
#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target robot_pai_hw" )
    add_clang_tooling(
            TARGETS robot_pai_hw
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS robot_pai_hw
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )


# install(FILES hit_robot_hw_sim_plugins.xml
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )