# CMake 版本要求，符合您 3.4-3.18 的范围
cmake_minimum_required(VERSION 3.5)

# 项目名称
project(patrol_car)

# C++ 标准和编译选项
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# -pthread 标志最好由 CMake 的 Threads 模块处理，但为保持原样，在此保留
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -mcpu=native -flto -pthread")
# 生成 compile_commands.json，便于编辑器和工具分析
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# --- 1. 设置源码和头文件目录 ---
# 将 src 目录的路径保存到一个变量中，方便后续使用
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)

# 添加头文件搜索路径
# 只需要包含存放 .hpp 和配置文件的目录
include_directories(
    ${SRC_DIR}/include
    ${SRC_DIR}/config
)


# --- 2. 查找外部依赖库 ---
# 使用 PkgConfig 查找库，这是个好习惯
find_package(PkgConfig REQUIRED)

# 查找 OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 查找 glib-2.0 (如果需要)
pkg_search_module(GLIB REQUIRED glib-2.0)
include_directories(${GLIB_INCLUDE_DIRS})
# 注意：在您的源码中并未看到使用glib，如果确实不用可以注释掉

# 查找 libserial
pkg_search_module(SERIAL REQUIRED libserial)
include_directories(${SERIAL_INCLUDE_DIRS})
# link_directories 是旧方法，pkg_search_module 找到的 ${SERIAL_LIBRARIES} 通常包含完整路径，无需额外设置

# 查找 ppnc
pkg_search_module(PPNC REQUIRED ppnc)
include_directories(${PPNC_INCLUDE_DIRS})

# 查找 onnx
# 注意：通常 onnxruntime 才有 .pc 文件，这里假设 'onnx' 是您系统上的一个有效 pkg-config 模块
pkg_search_module(ONNX REQUIRED onnx)
include_directories(${ONNX_INCLUDE_DIRS})


# --- 3. 定义源文件列表 ---

# 主程序 "icar" 的源文件列表
# 需要包含所有相关的 .cpp 文件才能成功链接
set(ICAR_SOURCES
    ${SRC_DIR}/icar.cpp
    ${SRC_DIR}/controlcenter.cpp
    ${SRC_DIR}/detection.cpp
    ${SRC_DIR}/mapping.cpp
    ${SRC_DIR}/motion.cpp
    ${SRC_DIR}/preprocess.cpp
    ${SRC_DIR}/thread.cpp
)

# 工具程序的源文件
set(CALIBRATION_SOURCES ${SRC_DIR}/tool/calibration.cpp)
set(CAMERA_SOURCES      ${SRC_DIR}/tool/camera.cpp)
# 假设 collection.cpp 和 img2video.cpp 存在于 tool/ 目录下
set(COLLECTION_SOURCES  ${SRC_DIR}/tool/collection.cpp)
set(IMG2VIDEO_SOURCES   ${SRC_DIR}/tool/img2video.cpp)

# 独立的 detection 测试程序 (如果需要)
# 注意：这会和 icar 程序里的 detection.cpp 重复编译，但遵循了您原始文件的意图
set(DETECTION_TEST_SOURCES ${SRC_DIR}/detection.cpp)


# --- 4. 创建可执行文件并链接库 ---

# 主程序: icar
add_executable(icar ${ICAR_SOURCES})
target_link_libraries(icar
    PRIVATE
    ${OpenCV_LIBS}
    ${PPNC_LIBRARIES}
    ${ONNX_LIBRARIES}
    ${SERIAL_LIBRARIES}
    # GLIB::GLIB # 如果使用 glib
    pthread # 明确链接 pthread
)

# 工具: calibration (相机标定)
add_executable(calibration ${CALIBRATION_SOURCES})
target_link_libraries(calibration PRIVATE ${OpenCV_LIBS} pthread)

# 工具: camera (相机测试)
add_executable(camera ${CAMERA_SOURCES})
target_link_libraries(camera PRIVATE ${OpenCV_LIBS} pthread)

# 工具: collection (图像采集)
add_executable(collection ${COLLECTION_SOURCES})
target_link_libraries(collection PRIVATE ${OpenCV_LIBS} ${SERIAL_LIBRARIES} pthread)

# 工具: img2video (图像合成视频)
add_executable(img2video ${IMG2VIDEO_SOURCES})
target_link_libraries(img2video PRIVATE ${OpenCV_LIBS} pthread)

# 独立测试程序: detection_test
# 为了避免与主程序中的同名源文件混淆，重命名可执行文件
add_executable(detection_test ${DETECTION_TEST_SOURCES})
target_link_libraries(detection_test
    PRIVATE
    ${OpenCV_LIBS}
    ${PPNC_LIBRARIES}
    ${ONNX_LIBRARIES}
    pthread
)

# --- 5. 安装和输出 (可选) ---
# 您可以设置安装路径，将生成的可执行文件统一存放
# set(CMAKE_INSTALL_PREFIX ${PROJECT_BINARY_DIR}/output)
# install(TARGETS icar calibration camera DESTINATION bin)