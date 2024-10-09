cmake_minimum_required(VERSION 3.5) # CMake install : https://cmake.org/download/
project(QtDemo LANGUAGES CXX)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_PREFIX_PATH "D:/Tools/QT/5.14.2/msvc2017_64") # Qt Kit Dir
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Qt5 COMPONENTS Widgets REQUIRED) # Qt COMPONENTS

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /Od") # 禁用优化（C）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Od") # 禁用优化（C++）# 设置编译器标志
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi") # 在 Release 模式下包含调试信息
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG") # 在 Release 模式下生成调试符号


set(RESOURCES_FILES resources/icon/logo.qrc)
set(RESOURCE_FILES ./resources/icon/resources.rc)

# OpenCV
set(OpenCV_DIR "D:/Tools/opencv/opencv/build") 
find_package(OpenCV REQUIRED)

aux_source_directory(./src srcs)


# Specify MSVC UTF-8 encoding   
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

add_executable(${PROJECT_NAME}
    WIN32 
    ${srcs}
    ${RESOURCES_FILES}
    ${RESOURCE_FILES}
) 
target_link_libraries(${PROJECT_NAME} PRIVATE Qt5::Widgets) # Qt5 Shared Library

# 链接 OpenCV 库
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

# 添加 OpenCV 的包含目录
target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

if(MSVC)
    #为编译器添加预定义的宏
    add_compile_definitions(_WINSOCK_DEPRECATED_NO_WARNINGS _CRT_SECURE_NO_WARNINGS)
    # 为 MSVC 添加 /utf-8 编译选项
    target_compile_options(${PROJECT_NAME} 
        PRIVATE 
        "/utf-8"
        "/Od"  # 禁用优化
        )
endif()
        