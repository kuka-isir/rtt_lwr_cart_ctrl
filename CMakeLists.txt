cmake_minimum_required(VERSION 2.8.3)
project(rtt_lwr_cart_imp_ctrl)

find_package(catkin REQUIRED COMPONENTS
    rtt_ros
    rtt_rosclock
    rtt_roscomm
    rtt_rosparam
    kuka_lwr_fri
    diagnostic_msgs
    geometry_msgs
    std_msgs
    kdl_conversions
    eigen_conversions
    lwr_fri
    rtt_lwr_abstract
    cmake_modules
    conman
    rtt_ros_kdl_tools
    lwr_flex_models
    nav_msgs
)


find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(INCLUDE_DIRS include)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
)

orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)


target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${kdl_conversions_LIBRARIES}
)


add_dependencies(${PROJECT_NAME} rtt_lwr_abstract)
orocos_install_headers(DIRECTORY include/${PROJECT_NAME})
orocos_generate_package(INCLUDE_DIRS include)
