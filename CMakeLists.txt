cmake_minimum_required(VERSION 3.5)
project(rplidar_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME}
        src/publisher.cpp
        )

ament_target_dependencies(${PROJECT_NAME}
        std_srvs
        rclcpp
        sensor_msgs
        )

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

add_executable(rplidarNode src/node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(rplidarNode ${PROJECT_NAME})

install(TARGETS
        rplidarNode
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS rplidarNode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()