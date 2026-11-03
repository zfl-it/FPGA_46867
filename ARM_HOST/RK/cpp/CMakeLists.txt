# cmake_minimum_required(VERSION 3.10)

# project(rknn_browser_hdmi_lpr_demo)

# option(DISABLE_RGA "Keep compatibility with build-linux.sh" OFF)
# option(DISABLE_LIBJPEG "Keep compatibility with build-linux.sh" OFF)
# option(USE_MODEL_ZOO_OPENCV "Use rknn_model_zoo bundled OpenCV" ON)

# if (ENABLE_ASAN)
#     message(STATUS "BUILD WITH ADDRESS SANITIZER")
#     set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
#     set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
#     set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
# endif()

# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# if(CMAKE_SIZEOF_VOID_P EQUAL 8)
#     set(TARGET_LIB_ARCH lib64)
# else()
#     set(TARGET_LIB_ARCH lib)
# endif()

# if(USE_MODEL_ZOO_OPENCV AND NOT DEFINED OpenCV_DIR)
#     if (CMAKE_SYSTEM_NAME STREQUAL "Android")
#         set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/opencv/opencv-android-sdk-build/sdk/native/jni/abi-${CMAKE_ANDROID_ARCH_ABI})
#     elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
#         set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/opencv/opencv-osx-x86_64/share/OpenCV)
#     elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
#         set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/opencv/opencv-windows-x86_64/x64/mingw/staticlib)
#     else()
#         if(TARGET_LIB_ARCH STREQUAL "lib")
#             set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/opencv/opencv-linux-armhf/share/OpenCV)
#         else()
#             set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/opencv/opencv-linux-aarch64/share/OpenCV)
#         endif()
#     endif()
# endif()

# find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs OPTIONAL_COMPONENTS freetype)
# message(STATUS OpenCV_DIR=${OpenCV_DIR})
# message(STATUS OpenCV_LIBS=${OpenCV_LIBS})

# find_package(PkgConfig QUIET)
# if(PkgConfig_FOUND)
#     pkg_check_modules(LIBDRM QUIET libdrm)
#     pkg_check_modules(FREETYPE2 QUIET freetype2)
# endif()
# if(NOT LIBDRM_FOUND)
#     set(LIBDRM_INCLUDE_DIRS /usr/include/libdrm /usr/include)
#     set(LIBDRM_LIBRARIES drm)
# endif()
# list(APPEND LIBDRM_INCLUDE_DIRS /usr/include)

# if(FREETYPE2_FOUND)
#     add_definitions(-DHAVE_DIRECT_FREETYPE=1)
# else()
#     set(FREETYPE2_INCLUDE_DIRS /usr/include/freetype2 /usr/include/libpng16)
#     if(EXISTS /usr/include/freetype2/ft2build.h)
#         add_definitions(-DHAVE_DIRECT_FREETYPE=1)
#     endif()
# endif()

# list(FIND OpenCV_LIBS opencv_freetype OPENCV_FREETYPE_INDEX)
# if(NOT OPENCV_FREETYPE_INDEX EQUAL -1)
#     add_definitions(-DHAVE_OPENCV_FREETYPE=1)
# endif()

# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/ 3rdparty.out)

# set(CMAKE_INSTALL_RPATH "$ORIGIN/lib")

# add_executable(${PROJECT_NAME}
#     main.cc
# )

# target_include_directories(${PROJECT_NAME} PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}
#     ${LIBRKNNRT_INCLUDES}
#     ${LIBDRM_INCLUDE_DIRS}
#     ${FREETYPE2_INCLUDE_DIRS}
# )

# target_link_libraries(${PROJECT_NAME}
#     ${LIBRKNNRT}
#     ${OpenCV_LIBS}
#     dl
# )

# if (CMAKE_SYSTEM_NAME STREQUAL "Android")
#     target_link_libraries(${PROJECT_NAME} log)
# endif()

# if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
#     set(THREADS_PREFER_PTHREAD_FLAG ON)
#     find_package(Threads REQUIRED)
#     target_link_libraries(${PROJECT_NAME} Threads::Threads)
# endif()

# install(TARGETS ${PROJECT_NAME} DESTINATION .)

# file(GLOB RKNN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../model/*.rknn")
# install(FILES ${RKNN_FILES} DESTINATION model OPTIONAL)

# set(DEFAULT_LPRNET_MODEL ${CMAKE_CURRENT_SOURCE_DIR}/../../LPRNet/model/lprnet.rknn)
# if(EXISTS ${DEFAULT_LPRNET_MODEL})
#     install(FILES ${DEFAULT_LPRNET_MODEL} DESTINATION model RENAME lprnet.rknn)
# endif()

# set(DEFAULT_FONT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../opencv_PRN_test/font/SimHei.ttf)
# if(EXISTS ${DEFAULT_FONT})
#     install(FILES ${DEFAULT_FONT} DESTINATION font)
# endif()



cmake_minimum_required(VERSION 3.10)

project(rknn_browser_hdmi_lpr_demo)

option(DISABLE_RGA "Keep compatibility with build-linux.sh" OFF)
option(DISABLE_LIBJPEG "Keep compatibility with build-linux.sh" OFF)
option(USE_MODEL_ZOO_OPENCV "Use rknn_model_zoo bundled OpenCV" ON)
option(ENABLE_DRM_ROI "Enable DRM ROI HDMI output path; libdrm is loaded at runtime" ON)
option(STATIC_LIBSTDCXX "Link libstdc++/libgcc statically for the main demo executable" OFF)

if (ENABLE_ASAN)
    message(STATUS "BUILD WITH ADDRESS SANITIZER")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(TARGET_LIB_ARCH lib64)
else()
    set(TARGET_LIB_ARCH lib)
endif()

if(USE_MODEL_ZOO_OPENCV AND NOT DEFINED OpenCV_DIR)
    if (CMAKE_SYSTEM_NAME STREQUAL "Android")
        set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/opencv/opencv-android-sdk-build/sdk/native/jni/abi-${CMAKE_ANDROID_ARCH_ABI})
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/opencv/opencv-osx-x86_64/share/OpenCV)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/opencv/opencv-windows-x86_64/x64/mingw/staticlib)
    else()
        if(TARGET_LIB_ARCH STREQUAL "lib")
            set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/opencv/opencv-linux-armhf/share/OpenCV)
        else()
            set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/opencv/opencv-linux-aarch64/share/OpenCV)
        endif()
    endif()
endif()

# 只使用 model_zoo 自带 OpenCV 的 core/imgproc/imgcodecs，不依赖 opencv_videoio。
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs OPTIONAL_COMPONENTS freetype)
message(STATUS OpenCV_DIR=${OpenCV_DIR})
message(STATUS OpenCV_LIBS=${OpenCV_LIBS})

if(DEFINED ENV{ZLM_SDK_DIR})
    set(ZLM_SDK_DIR "$ENV{ZLM_SDK_DIR}" CACHE PATH "ZLMediaKit C API SDK directory" FORCE)
else()
    set(ZLM_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ZLMediaKit" CACHE PATH "ZLMediaKit C API SDK directory")
endif()
if(DEFINED ENV{ZLM_INCLUDE_DIR})
    set(ZLM_INCLUDE_DIR "$ENV{ZLM_INCLUDE_DIR}" CACHE PATH "ZLMediaKit C API include directory" FORCE)
endif()
if(DEFINED ENV{ZLM_LIBRARY})
    set(ZLM_LIBRARY "$ENV{ZLM_LIBRARY}" CACHE FILEPATH "ZLMediaKit libmk_api.so path" FORCE)
endif()
set(ZLM_INCLUDE_CANDIDATES
    "${ZLM_SDK_DIR}/api/include"
    "${ZLM_SDK_DIR}/include"
    "${ZLM_SDK_DIR}/release/linux/Debug/include"
    "${ZLM_SDK_DIR}/release/linux/Release/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ZLMediaKit/api/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ZLMediaKit/include"
)
set(ZLM_LIBRARY_CANDIDATES
    "${ZLM_SDK_DIR}/release/linux/Debug/libmk_api.so"
    "${ZLM_SDK_DIR}/release/linux/Release/libmk_api.so"
    "${ZLM_SDK_DIR}/lib/libmk_api.so"
    "${ZLM_SDK_DIR}/libmk_api.so"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ZLMediaKit/release/linux/Debug/libmk_api.so"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ZLMediaKit/release/linux/Release/libmk_api.so"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ZLMediaKit/lib/libmk_api.so"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ZLMediaKit/libmk_api.so"
)

foreach(_zlm_inc ${ZLM_INCLUDE_CANDIDATES})
    if(NOT ZLM_INCLUDE_DIR AND EXISTS "${_zlm_inc}/mk_mediakit.h")
        set(ZLM_INCLUDE_DIR "${_zlm_inc}")
    endif()
endforeach()

foreach(_zlm_lib ${ZLM_LIBRARY_CANDIDATES})
    if(NOT ZLM_LIBRARY AND EXISTS "${_zlm_lib}")
        set(ZLM_LIBRARY "${_zlm_lib}")
    endif()
endforeach()

if(NOT ZLM_INCLUDE_DIR)
    message(FATAL_ERROR "ZLMediaKit C API header not found. Checked: ${ZLM_INCLUDE_CANDIDATES}")
endif()
if(NOT ZLM_LIBRARY)
    message(FATAL_ERROR "ZLMediaKit libmk_api.so not found. Checked: ${ZLM_LIBRARY_CANDIDATES}")
endif()
get_filename_component(ZLM_LIBRARY_DIR "${ZLM_LIBRARY}" DIRECTORY)
message(STATUS "ZLMediaKit include: ${ZLM_INCLUDE_DIR}")
message(STATUS "ZLMediaKit lib: ${ZLM_LIBRARY}")

set(ZLM_CONFIG_INI "" CACHE FILEPATH "ZLMediaKit config.ini path for install packaging")
set(ZLM_CONFIG_CANDIDATES
    "${ZLM_SDK_DIR}/config.ini"
    "${ZLM_SDK_DIR}/conf/config.ini"
    "${CMAKE_CURRENT_SOURCE_DIR}/ZLMediaKit/config.ini"
    "${CMAKE_CURRENT_SOURCE_DIR}/ZLMediaKit/conf/config.ini"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ZLMediaKit/conf/config.ini"
)
foreach(_zlm_cfg ${ZLM_CONFIG_CANDIDATES})
    if(NOT ZLM_CONFIG_INI AND EXISTS "${_zlm_cfg}")
        set(ZLM_CONFIG_INI "${_zlm_cfg}" CACHE FILEPATH "ZLMediaKit config.ini path for install packaging" FORCE)
    endif()
endforeach()
if(ZLM_CONFIG_INI)
    message(STATUS "ZLMediaKit config.ini for install: ${ZLM_CONFIG_INI}")
    file(READ "${ZLM_CONFIG_INI}" ZLM_RUNTIME_CONFIG_TEXT)
    foreach(_zlm_disabled_output enable_hls enable_hls_fmp4 enable_mp4 enable_fmp4)
        string(REGEX REPLACE "(^|\n)${_zlm_disabled_output}=[^\n]*"
            "\\1${_zlm_disabled_output}=0"
            ZLM_RUNTIME_CONFIG_TEXT
            "${ZLM_RUNTIME_CONFIG_TEXT}")
    endforeach()
    set(ZLM_RUNTIME_CONFIG_INI "${CMAKE_CURRENT_BINARY_DIR}/zlm_runtime_config.ini")
    file(WRITE "${ZLM_RUNTIME_CONFIG_INI}" "${ZLM_RUNTIME_CONFIG_TEXT}")
    message(STATUS "ZLMediaKit packaged config disables HLS/MP4/FMP4 outputs; WebRTC/HTTP-FLV/RTSP remain enabled.")
else()
    message(WARNING "ZLMediaKit config.ini not found for install packaging. Runtime default ./ZLMediaKit/config.ini must be provided.")
endif()

if(DEFINED ENV{MPP_INCLUDE_DIR})
    set(MPP_INCLUDE_DIR "$ENV{MPP_INCLUDE_DIR}" CACHE PATH "Rockchip MPP include directory" FORCE)
endif()
if(DEFINED ENV{MPP_LIBRARY})
    set(MPP_LIBRARY "$ENV{MPP_LIBRARY}" CACHE FILEPATH "Rockchip MPP library path" FORCE)
endif()
find_path(MPP_INCLUDE_DIR
    NAMES rockchip/rk_mpi.h
    PATHS /usr/include /usr/local/include
)
find_library(MPP_LIBRARY
    NAMES rockchip_mpp mpp
    PATHS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu /usr/lib64
)
if(NOT MPP_LIBRARY)
    find_file(MPP_LIBRARY
        NAMES librockchip_mpp.so.0 libmpp.so.0
        PATHS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu /usr/lib64
    )
endif()
if(NOT MPP_INCLUDE_DIR)
    message(FATAL_ERROR "Rockchip MPP header rockchip/rk_mpi.h not found. Install MPP development headers or set MPP_INCLUDE_DIR.")
endif()
if(NOT MPP_LIBRARY)
    message(FATAL_ERROR "Rockchip MPP library not found. Tried rockchip_mpp/mpp in system library paths; set MPP_LIBRARY if needed.")
endif()
message(STATUS "Rockchip MPP include: ${MPP_INCLUDE_DIR}")
message(STATUS "Rockchip MPP lib: ${MPP_LIBRARY}")

if(DEFINED ENV{RGA_INCLUDE_DIR})
    set(RGA_INCLUDE_DIR "$ENV{RGA_INCLUDE_DIR}" CACHE PATH "RGA include directory" FORCE)
endif()
if(DEFINED ENV{RGA_LIBRARY})
    set(RGA_LIBRARY "$ENV{RGA_LIBRARY}" CACHE FILEPATH "RGA library path" FORCE)
endif()
set(RGA_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/librga")
find_path(RGA_INCLUDE_DIR
    NAMES im2d.h
    PATHS "${RGA_ROOT}/include" /usr/include/rga /usr/local/include/rga /usr/include /usr/local/include
)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(RGA_ARCH_LIB_DIR "${RGA_ROOT}/Linux/aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    set(RGA_ARCH_LIB_DIR "${RGA_ROOT}/Linux/armhf")
else()
    set(RGA_ARCH_LIB_DIR "")
endif()
find_library(RGA_LIBRARY
    NAMES rga
    PATHS "${RGA_ARCH_LIB_DIR}" /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu /usr/lib64
)
if(RGA_INCLUDE_DIR AND RGA_LIBRARY)
    message(STATUS "RGA include: ${RGA_INCLUDE_DIR}")
    message(STATUS "RGA lib: ${RGA_LIBRARY}")
else()
    message(WARNING "RGA headers/library not found; pixel_convert.cc will use OpenCV CPU BGR->NV12 fallback.")
endif()

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(LIBDRM QUIET libdrm)
    pkg_check_modules(FREETYPE2 QUIET freetype2)
endif()

if(NOT LIBDRM_FOUND)
    set(LIBDRM_INCLUDE_DIRS /usr/include/libdrm /usr/include)
    set(LIBDRM_LIBRARIES drm)
endif()
list(APPEND LIBDRM_INCLUDE_DIRS /usr/include)

if(FREETYPE2_FOUND)
    set(HAVE_FREETYPE_FLAG 1)
else()
    set(FREETYPE2_INCLUDE_DIRS /usr/include/freetype2 /usr/include/libpng16)
    if(EXISTS /usr/include/freetype2/ft2build.h)
        set(HAVE_FREETYPE_FLAG 1)
    endif()
endif()

list(FIND OpenCV_LIBS opencv_freetype OPENCV_FREETYPE_INDEX)
if(NOT OPENCV_FREETYPE_INDEX EQUAL -1)
    set(HAVE_OPENCV_FREETYPE_FLAG 1)
endif()

if(DEFINED ENV{LIBRKNNRT})
    set(LIBRKNNRT "$ENV{LIBRKNNRT}" CACHE FILEPATH "RKNN runtime library" FORCE)
endif()
if(DEFINED ENV{LIBRKNNRT_INCLUDES})
    set(LIBRKNNRT_INCLUDES "$ENV{LIBRKNNRT_INCLUDES}" CACHE PATH "RKNN runtime include directory" FORCE)
endif()
if(NOT LIBRKNNRT OR NOT LIBRKNNRT_INCLUDES)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/ 3rdparty.out)
endif()

set(CMAKE_INSTALL_RPATH "$ORIGIN/lib")

add_executable(${PROJECT_NAME}
    main.cc
    adas_detector.cc
    zlm_streamer.cc
    mpp_h264_encoder.cc
    pixel_convert.cc
    pango_fun.cpp
)

add_executable(pcie_reader
    main.c
)

if(STATIC_LIBSTDCXX)
    set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " -static-libstdc++ -static-libgcc")
    message(STATUS "STATIC_LIBSTDCXX=ON for ${PROJECT_NAME}")
endif()

if(ENABLE_DRM_ROI)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_DRM_ROI=1)
    message(STATUS "HAVE_DRM_ROI=1 enabled for ${PROJECT_NAME}")
else()
    message(STATUS "HAVE_DRM_ROI disabled.")
endif()

if(HAVE_FREETYPE_FLAG)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_DIRECT_FREETYPE=1)
endif()

if(HAVE_OPENCV_FREETYPE_FLAG)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_OPENCV_FREETYPE=1)
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ZLM_INCLUDE_DIR}
    ${MPP_INCLUDE_DIR}
    ${LIBRKNNRT_INCLUDES}
    ${LIBDRM_INCLUDE_DIRS}
    ${FREETYPE2_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    ${ZLM_LIBRARY}
    ${MPP_LIBRARY}
    ${LIBRKNNRT}
    ${OpenCV_LIBS}
    dl
    m
)

if(RGA_INCLUDE_DIR AND RGA_LIBRARY)
    target_include_directories(${PROJECT_NAME} PRIVATE ${RGA_INCLUDE_DIR})
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_RGA=1)
    target_link_libraries(${PROJECT_NAME} ${RGA_LIBRARY})
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "${ZLM_LIBRARY_DIR}"
    INSTALL_RPATH "$ORIGIN/lib;${ZLM_LIBRARY_DIR}"
)
message(STATUS "Runtime hint: export LD_LIBRARY_PATH=${ZLM_LIBRARY_DIR}:$LD_LIBRARY_PATH")

if (CMAKE_SYSTEM_NAME STREQUAL "Android")
    target_link_libraries(${PROJECT_NAME} log)
endif()

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    target_link_libraries(${PROJECT_NAME} Threads::Threads)
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION .)
install(TARGETS pcie_reader DESTINATION .)
install(FILES ${ZLM_LIBRARY} DESTINATION lib)
install(FILES ${ZLM_LIBRARY} DESTINATION ZLMediaKit/lib)
install(DIRECTORY "${ZLM_INCLUDE_DIR}/" DESTINATION ZLMediaKit/include FILES_MATCHING PATTERN "*.h")
if(ZLM_RUNTIME_CONFIG_INI)
    install(FILES "${ZLM_RUNTIME_CONFIG_INI}" DESTINATION ZLMediaKit RENAME config.ini)
endif()
install(FILES ${MPP_LIBRARY} DESTINATION lib)
get_filename_component(MPP_LIBRARY_NAME "${MPP_LIBRARY}" NAME)
get_filename_component(MPP_LIBRARY_DIR "${MPP_LIBRARY}" DIRECTORY)
if(MPP_LIBRARY_NAME STREQUAL "librockchip_mpp.so.0")
    install(FILES ${MPP_LIBRARY} DESTINATION lib RENAME librockchip_mpp.so.1)
endif()
if(EXISTS "${MPP_LIBRARY_DIR}/librockchip_mpp.so.1")
    install(FILES "${MPP_LIBRARY_DIR}/librockchip_mpp.so.1" DESTINATION lib)
endif()
if(EXISTS "${LIBRKNNRT}")
    install(FILES ${LIBRKNNRT} DESTINATION lib)
endif()
if(RGA_LIBRARY AND EXISTS "${RGA_LIBRARY}")
    install(FILES ${RGA_LIBRARY} DESTINATION lib)
    get_filename_component(RGA_LIBRARY_DIR "${RGA_LIBRARY}" DIRECTORY)
    if(EXISTS "${RGA_LIBRARY_DIR}/librga.so.2")
        install(FILES "${RGA_LIBRARY_DIR}/librga.so.2" DESTINATION lib)
    endif()
endif()

file(GLOB RKNN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../model/*.rknn")
install(FILES ${RKNN_FILES} DESTINATION model OPTIONAL)

set(YOLOV5PLATE_FP_MODEL ${CMAKE_CURRENT_SOURCE_DIR}/../python/yolov5plate_fp.rknn)
if(EXISTS ${YOLOV5PLATE_FP_MODEL})
    install(FILES ${YOLOV5PLATE_FP_MODEL} DESTINATION model RENAME yolov5plate_fp.rknn)
endif()

set(PPOCR_KEYS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../model/ppocr_keys_v1.txt)
if(EXISTS ${PPOCR_KEYS_FILE})
    install(FILES ${PPOCR_KEYS_FILE} DESTINATION model)
endif()

set(DEFAULT_LPRNET_MODEL ${CMAKE_CURRENT_SOURCE_DIR}/../../LPRNet/model/lprnet.rknn)
if(EXISTS ${DEFAULT_LPRNET_MODEL})
    install(FILES ${DEFAULT_LPRNET_MODEL} DESTINATION model RENAME lprnet.rknn)
endif()

set(DEFAULT_FONT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../opencv_PRN_test/font/SimHei.ttf)
if(EXISTS ${DEFAULT_FONT})
    install(FILES ${DEFAULT_FONT} DESTINATION font)
endif()
