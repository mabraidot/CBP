cmake_minimum_required(VERSION 3.7)
project(CBP)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        Libraries/DCMotorServo/DCMotorServo.cpp
        Libraries/DCMotorServo/DCMotorServo.h
        Libraries/DCMotorServo_v1/DCMotorServo.cpp
        Libraries/DCMotorServo_v1/DCMotorServo.h
        Libraries/SpeedSensor/SpeedSensor.cpp
        Libraries/SpeedSensor/SpeedSensor.h
        Configuration.h
        DCMotorServo.cpp
        DCMotorServo.h
        Planner.cpp
        Planner.h
        SpeedSensor.cpp
        SpeedSensor.h)

add_executable(CBP ${SOURCE_FILES})

set(${CMAKE_PROJECT_NAME}_BOARD uno)
set(${CMAKE_PROJECT_NAME}_PORT /dev/ttys0)
LINK_DIRECTORIES(C:\\Users\\Miguel\\Documents\\Arduino\\libraries)