cmake_minimum_required(VERSION 2.8.3)

project(spatial_audio)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    audio_common_msgs
    spatial_audio_msgs
    geometry_msgs
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
)

find_package(Boost REQUIRED)
find_package(OpenAL REQUIRED)

catkin_python_setup()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
        audio_common_msgs
        spatial_audio_msgs
        geometry_msgs
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
    DEPENDS Boost
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OPENAL_INCLUDE_DIR}
)

add_executable(
    spatial_audio_server_node
    src/spatial_audio_source.cpp
    src/spatial_audio_server.cpp
    src/spatial_audio_server_node.cpp
    src/util.cpp
)
target_link_libraries(
    spatial_audio_server_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OPENAL_LIBRARY}
)
target_compile_definitions(
    spatial_audio_server_node
    PRIVATE
    AL_ALEXT_PROTOTYPES
)
