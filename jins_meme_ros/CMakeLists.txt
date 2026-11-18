cmake_minimum_required(VERSION 3.0.2)
project(jins_meme_ros)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
    FILES
    CurrentData.msg
    LogicIndexData.msg
)

generate_messages()

catkin_package(
    CATKIN_DEPENDS message_runtime
)
