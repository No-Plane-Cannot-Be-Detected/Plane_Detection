
project(Point-Cloud-Plane-Detection)
set(project_name Point-Cloud-Plane-Detection)
cmake_minimum_required(VERSION 3.2)

IF (CMAKE_SYSTEM_NAME MATCHES "Windows")
    message("Windows")

    set(OpenCV_DIR "D:/software/Environment/opencv/build/x64/vc15/lib")

    find_package(OpenCV REQUIRED)

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")

    ## 信息输出(非必须)
    message(STATUS "OpenCV library status:")
    message(STATUS "    config: ${OpenCV_DIR}")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    libraries: ${OpenCV_LIBS}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

    include_directories(include ${OpenCV_INCLUDE_DIRS})

    add_executable(Point-Cloud-Plane-Detection source/main.cpp include/ransac.h source/ransac.cpp include/utils.h source/utils.cpp)

    target_link_libraries(Point-Cloud-Plane-Detection ${OpenCV_LIBS})

    target_include_directories(Point-Cloud-Plane-Detection PUBLIC)

ELSEIF (CMAKE_SYSTEM_NAME MATCHES "Linux")
    message("Linux")

    find_package(OpenCV REQUIRED)
    include_directories(include ${OpenCV_INCLUDE_DIRS})

    add_executable(Point-Cloud-Plane-Detection source/main.cpp include/ransac.h source/ransac.cpp include/utils.h source/utils.cpp)
    #    add_executable(Point-Cloud-Plane-Detection source/main.cpp)
    target_link_libraries(Point-Cloud-Plane-Detection ${OpenCV_LIBS})

ENDIF ()

