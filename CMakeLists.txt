cmake_minimum_required(VERSION 3.15)
project(UR5SimulationForceEstimation)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Debug")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin")

find_package(Boost REQUIRED COMPONENTS program_options iostreams system filesystem)
find_package(Eigen3 REQUIRED)

add_library(gnuplot_iostream INTERFACE)
target_include_directories(gnuplot_iostream INTERFACE "${CMAKE_SOURCE_DIR}/gnuplot-iostream")
target_link_libraries(gnuplot_iostream INTERFACE
    Boost::iostreams
    Boost::system
    Boost::filesystem
)

set(ExtObserverLibInclude "${CMAKE_SOURCE_DIR}/include/ExtObserverLib_v3")
set(ExtObserverLibSrc "${CMAKE_SOURCE_DIR}/src/ExtObserverLib_v3")

# === 定义 Observer 库源文件 (通用) ===
set(OBSERVER_LIB_SOURCES
    ${ExtObserverLibSrc}/external_observer_rnea.cpp
    ${ExtObserverLibSrc}/momentum_observer_rnea.cpp
    ${ExtObserverLibSrc}/filtered_dyn_observer_rnea.cpp
    ${ExtObserverLibSrc}/nonlinear_disturbance_observer_rnea.cpp
    ${ExtObserverLibSrc}/sliding_mode_observer_rnea.cpp
    ${ExtObserverLibSrc}/kalman_filter_observer_rnea.cpp
    ${ExtObserverLibSrc}/kalman_filter_zero_order.cpp
    ${ExtObserverLibSrc}/kalman_filter_zero_order_rnea.cpp
)

# =========================================================
# 1. 原有的 UR5 配置
# =========================================================

# 【修正】这里必须指向 UR5Simulation 目录，而不是 RM65
file(GLOB UR5_DIR_SRCS "src/UR5Simulation/*.cpp") 

set(UR5_SOURCES
    ${OBSERVER_LIB_SOURCES}
    ${UR5_DIR_SRCS}
    src/UR5EstimateForceEstimation.cpp)

add_executable(UR5EstimateForceEstimation ${UR5_SOURCES})
target_include_directories(UR5EstimateForceEstimation PUBLIC 
    ${ExtObserverLibInclude} 
    PUBLIC include 
    PUBLIC ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(UR5EstimateForceEstimation PUBLIC Boost::program_options gnuplot_iostream)

# =========================================================
# 2. 新增 RM65_Test 配置
# =========================================================

# 定义 RM65 的相关路径
set(RM65_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include/RM65Simulation")
set(RM65_SRC_DIR "${CMAKE_SOURCE_DIR}/src/RM65Simulation")

# 自动搜索 RM65 目录下的所有 cpp 文件
file(GLOB RM65_DIR_SRCS "${RM65_SRC_DIR}/*.cpp")

set(RM65_SOURCES
    ${OBSERVER_LIB_SOURCES}
    src/RM65_Test_final.cpp
    ${RM65_DIR_SRCS})

# 添加 RM65 测试可执行文件
add_executable(RM65_Test ${RM65_SOURCES})

# 配置头文件路径
target_include_directories(RM65_Test PUBLIC 
    ${ExtObserverLibInclude} 
    ${RM65_INCLUDE_DIR}
    PUBLIC include 
    PUBLIC ${EIGEN3_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(RM65_Test PUBLIC Boost::program_options gnuplot_iostream)

# =========================================================
# 3. 新增 RM65 验证程序 (C++ Direct Verification)
# =========================================================

set(RM65_VERIFICATION_SOURCES
    src/RM65_Verification.cpp
    ${RM65_DIR_SRCS}       # 包含 RM65Simulation 下的所有 cpp
    ${OBSERVER_LIB_SOURCES} # 包含 ExternalObserverRnea 等基类
)

add_executable(RM65_Verification ${RM65_VERIFICATION_SOURCES})

target_include_directories(RM65_Verification PUBLIC 
    ${ExtObserverLibInclude} 
    ${RM65_INCLUDE_DIR}
    PUBLIC include 
    PUBLIC ${EIGEN3_INCLUDE_DIRS}
)

# 链接 boost 和 gnuplot
target_link_libraries(RM65_Verification PUBLIC Boost::program_options gnuplot_iostream)
