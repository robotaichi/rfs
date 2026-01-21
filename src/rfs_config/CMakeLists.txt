cmake_minimum_required(VERSION 3.8)
project(rfs_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
    DESTINATION share/rfs_config
)

ament_package()
