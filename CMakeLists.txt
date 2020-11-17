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

add_executable(rplidarNode src/node.cc)
target_link_libraries(rplidarNode ${PROJECT_NAME})

install(
        DIRECTORY include/
        DESTINATION include
)

install(TARGETS
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

set(GET_PORT_FILEPATH ${CMAKE_CURRENT_SOURCE_DIR}/../../install/${PROJECT_NAME}/share/${PROJECT_NAME}/scripts)

install(
        FILES ${GET_PORT_FILEPATH}/tmp/get_serial_port.py
        PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
        DESTINATION ${GET_PORT_FILEPATH}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()