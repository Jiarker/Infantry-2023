cmake_minimum_required(VERSION 3.10)
project(AIOF)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPATH=\"\\\"${PROJECT_SOURCE_DIR}\\\"\"")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D${CMAKE_SYSTEM_NAME}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#查找必要的依赖包
find_package(OpenCV 4 REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(ngraph REQUIRED)

add_library(aiof SHARED
    src/Module.cc
)

target_link_libraries(aiof
    ${OpenCV_LIBS}
    ${InferenceEngine_LIBRARIES}
    ${NGRAPH_LIBRARIES}
    # mercure_driver
)

target_include_directories(aiof
    PUBLIC ${OpenCV_INCLUDE_DIR}
    PUBLIC ${InferenceEngine_INCLUDE_DIRS}
    PUBLIC include
)



# Install libraries
install(TARGETS aiof
    EXPORT aiof
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)