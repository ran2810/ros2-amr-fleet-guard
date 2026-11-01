cmake_minimum_required(VERSION 3.8)
project(warehouse_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps 
        DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/send_goal.py 
        DESTINATION lib/${PROJECT_NAME})

ament_package()
