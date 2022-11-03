cmake_minimum_required(VERSION 3.10)
project(rm_fsm)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -Wno-address-of-packed-member)

## Find catkin macros and libraries

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        sensor_msgs
        roslint
        rm_msgs
        std_msgs
        nav_msgs
        rm_common
        tf
        dynamic_reconfigure
        controller_manager
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        sensor_msgs
        roslint
        rm_msgs
        std_msgs
        nav_msgs
        rm_common
        tf
        dynamic_reconfigure
        controller_manager
        DEPENDS
)

###########
## Build ##
###########

set(SMC ${PROJECT_SOURCE_DIR}/lib/smc/bin/Smc.jar)
set(SM_CLASS_NAME StateMachine)
set(SM_SOURCES ${PROJECT_SOURCE_DIR}/state_machine/${SM_CLASS_NAME}.sm)

add_custom_target(${PROJECT_NAME}_generate_sm_file
        COMMAND java -jar ${SMC} -c++ ${SM_SOURCES}
        COMMENT "Generating State Machine file")
add_custom_target(${PROJECT_NAME}_generate_sm_graph
        COMMAND java -jar ${SMC} -graph -glevel 1 ${SM_SOURCES}
        COMMAND dot -T png ${PROJECT_SOURCE_DIR}/state_machine/${SM_CLASS_NAME}_sm.dot -o ${PROJECT_SOURCE_DIR}/state_machine/${SM_CLASS_NAME}_sm.png
        COMMENT "Generating State Machine graph")
add_custom_target(${PROJECT_NAME}_generate_sm_table
        COMMAND java -jar ${SMC} -table ${SM_SOURCES}
        COMMENT "Generating State Machine graph")
add_dependencies(${PROJECT_NAME}_generate_sm_graph ${PROJECT_NAME}_generate_sm_file)
add_dependencies(${PROJECT_NAME}_generate_sm_table ${PROJECT_NAME}_generate_sm_file)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        lib/smc/C++
        state_machine
        ${catkin_INCLUDE_DIRS}
)


## Declare cpp executables
FILE(GLOB ALL_SOURCES "src/*.cpp" "state_machine/*.cpp")
add_executable(${PROJECT_NAME} ${ALL_SOURCES})

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
        DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslint_cpp()
