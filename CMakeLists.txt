cmake_minimum_required(VERSION 3.8)
project(controller)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        data/BMA180-DataSheet-v2.5.pdf
        data/i2c.txt
        data/MultiWiiPinout.jpg
        controller.ino
        i2c.txt)

add_executable(controller ${SOURCE_FILES})