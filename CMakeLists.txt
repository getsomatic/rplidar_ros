cmake_minimum_required(VERSION 3.5)
project(rplidar_ros)
add_compile_options(-std=c++14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(RPLIDAR_SDK_PATH "./sdk/")
FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

include_directories(
    ${RPLIDAR_SDK_PATH}/include
    ${RPLIDAR_SDK_PATH}/src
)

add_library(${PROJECT_NAME}
        ${RPLIDAR_SDK_SRC}
        src/publisher.cc
        )

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        std_srvs
        sensor_msgs
        ament_index_cpp
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_executable(rplidarNode1 src/node1.cc)
target_link_libraries(rplidarNode1 ${PROJECT_NAME})

add_executable(rplidarNode2 src/node2.cc)
target_link_libraries(rplidarNode2 ${PROJECT_NAME})

add_executable(rplidarNode3 src/node3.cc)
target_link_libraries(rplidarNode3 ${PROJECT_NAME})

add_executable(rplidarNode src/node.cc)
target_link_libraries(rplidarNode ${PROJECT_NAME})

install(
        DIRECTORY include/
        DESTINATION include
)

install(TARGETS
        rplidarNode1
        rplidarNode2
        rplidarNode3
        rplidarNode
        DESTINATION
        lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
    INCLUDES DESTINATION include
)

install(DIRECTORY
        launch
        config
        scripts
        DESTINATION share/${PROJECT_NAME}/
        )

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()