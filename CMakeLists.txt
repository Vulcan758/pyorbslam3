cmake_minimum_required(VERSION 3.16)
project(pyorbslam3 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Reuse the ORB-SLAM3 build produced by the old ORB-SLAM-Python project.
# Everything there was compiled with -fPIC (see cmakefix.patch), so the static
# libraries can be linked straight into this Python extension module.
set(ORBSLAM_DEPS
    "${CMAKE_CURRENT_SOURCE_DIR}/../ORB-SLAM-Python/build/cp310-cp310-linux_x86_64/_deps"
    CACHE PATH "Path to the _deps dir of the existing ORB-SLAM-Python build")

set(ORBSLAM_SRC "${ORBSLAM_DEPS}/orb_slam3-src")
set(ORBSLAM_BUILD "${ORBSLAM_DEPS}/orb_slam3-build")
set(PANGOLIN_SRC "${ORBSLAM_DEPS}/pangolin-src")
set(PANGOLIN_BUILD "${ORBSLAM_DEPS}/pangolin-build")

if(NOT EXISTS "${ORBSLAM_BUILD}/libORB_SLAM3.a")
  message(FATAL_ERROR "libORB_SLAM3.a not found under ${ORBSLAM_BUILD}. "
                      "Set -DORBSLAM_DEPS=<path to _deps of the ORB-SLAM-Python build>.")
endif()

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 REQUIRED)

pybind11_add_module(_core MODULE src/orbslam3_pybind.cpp)

file(GLOB PANGOLIN_COMPONENT_INCLUDES "${PANGOLIN_SRC}/components/*/include")

target_include_directories(
  _core SYSTEM
  PRIVATE "${ORBSLAM_SRC}"
          "${ORBSLAM_SRC}/include"
          "${ORBSLAM_SRC}/include/CameraModels"
          "${ORBSLAM_SRC}/Thirdparty/Sophus"
          ${PANGOLIN_COMPONENT_INCLUDES}
          "${PANGOLIN_BUILD}/include"
          ${OpenCV_INCLUDE_DIRS})

target_compile_definitions(
  _core PRIVATE HAVE_GLEW HAVE_EIGEN _LINUX_ PANGO_DEFAULT_WIN_URI="wayland"
                BOOST_ALL_NO_LIB BOOST_SERIALIZATION_DYN_LINK)

file(GLOB PANGOLIN_LIBS "${PANGOLIN_BUILD}/libpango_*.a")

target_link_libraries(
  _core
  PRIVATE
    -Wl,--start-group
    "${ORBSLAM_BUILD}/libORB_SLAM3.a"
    "${ORBSLAM_BUILD}/Thirdparty/DBoW2/libDBoW2.a"
    "${ORBSLAM_BUILD}/Thirdparty/g2o/libg2o.a"
    ${PANGOLIN_LIBS}
    "${PANGOLIN_BUILD}/libtinyobj.a"
    -Wl,--end-group
    Eigen3::Eigen
    ${OpenCV_LIBS}
    boost_serialization
    crypto
    GLEW
    OpenGL
    GLX
    GLU
    EGL
    SM
    ICE
    X11
    Xext
    wayland-client
    wayland-egl
    wayland-cursor
    xkbcommon
    rt)

# Drop the built module straight into the Python package.
set_target_properties(_core PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/pyorbslam3")
