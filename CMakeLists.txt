cmake_minimum_required(VERSION 3.16)
project(pyorbslam3 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Where does ORB-SLAM3 come from?
#
#  (a) ORBSLAM3_ROOT  — a standalone ORB_SLAM3 source tree that has been built
#                       in <root>/build (libORB_SLAM3.a + Thirdparty libs), with
#                       Pangolin discoverable through find_package(Pangolin).
#                       This is what the Dockerfile uses.
#  (b) ORBSLAM_DEPS   — legacy: the _deps dir of a PRBonn ORB-SLAM-Python
#                       build (orb_slam3-src/-build, pangolin-src/-build).
#
# (a) is picked when ORBSLAM3_ROOT is given or ../ORB_SLAM3/build exists.
# ---------------------------------------------------------------------------
set(ORBSLAM3_ROOT "" CACHE PATH "Standalone ORB_SLAM3 source tree (built in <root>/build)")
set(ORBSLAM_DEPS
    "${CMAKE_CURRENT_SOURCE_DIR}/../ORB-SLAM-Python/build/cp310-cp310-linux_x86_64/_deps"
    CACHE PATH "Legacy: _deps dir of an ORB-SLAM-Python build")

if(ORBSLAM3_ROOT STREQUAL "" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3/build/libORB_SLAM3.a")
  set(ORBSLAM3_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3")
endif()

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)

pybind11_add_module(_core MODULE src/orbslam3_pybind.cpp)

target_compile_definitions(
  _core PRIVATE HAVE_GLEW HAVE_EIGEN _LINUX_ PANGO_DEFAULT_WIN_URI="wayland"
                BOOST_ALL_NO_LIB BOOST_SERIALIZATION_DYN_LINK)

if(NOT ORBSLAM3_ROOT STREQUAL "")
  # ---- (a) standalone tree ------------------------------------------------
  set(ORBSLAM3_BUILD "${ORBSLAM3_ROOT}/build")
  if(NOT EXISTS "${ORBSLAM3_BUILD}/libORB_SLAM3.a")
    message(FATAL_ERROR "libORB_SLAM3.a not found in ${ORBSLAM3_BUILD}; build ORB_SLAM3 first.")
  endif()
  find_package(Pangolin REQUIRED)
  message(STATUS "pyorbslam3: linking standalone ORB_SLAM3 at ${ORBSLAM3_ROOT}")

  target_include_directories(
    _core SYSTEM
    PRIVATE "${ORBSLAM3_ROOT}"
            "${ORBSLAM3_ROOT}/include"
            "${ORBSLAM3_ROOT}/include/CameraModels"
            "${ORBSLAM3_ROOT}/Thirdparty/Sophus"
            ${Pangolin_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS})

  target_link_libraries(
    _core
    PRIVATE
      -Wl,--start-group
      "${ORBSLAM3_BUILD}/libORB_SLAM3.a"
      "${ORBSLAM3_BUILD}/Thirdparty/DBoW2/libDBoW2.a"
      "${ORBSLAM3_BUILD}/Thirdparty/g2o/libg2o.a"
      ${Pangolin_LIBRARIES}
      -Wl,--end-group
      Eigen3::Eigen
      ${OpenCV_LIBS}
      Boost::serialization
      OpenSSL::Crypto)
else()
  # ---- (b) legacy ORB-SLAM-Python _deps tree ------------------------------
  set(ORBSLAM_SRC "${ORBSLAM_DEPS}/orb_slam3-src")
  set(ORBSLAM_BUILD "${ORBSLAM_DEPS}/orb_slam3-build")
  set(PANGOLIN_SRC "${ORBSLAM_DEPS}/pangolin-src")
  set(PANGOLIN_BUILD "${ORBSLAM_DEPS}/pangolin-build")
  if(NOT EXISTS "${ORBSLAM_BUILD}/libORB_SLAM3.a")
    message(FATAL_ERROR "No ORB-SLAM3 build found. Set -DORBSLAM3_ROOT=<standalone tree> "
                        "or -DORBSLAM_DEPS=<_deps of an ORB-SLAM-Python build>.")
  endif()
  message(STATUS "pyorbslam3: linking legacy ORB-SLAM-Python deps at ${ORBSLAM_DEPS}")

  file(GLOB PANGOLIN_COMPONENT_INCLUDES "${PANGOLIN_SRC}/components/*/include")
  file(GLOB PANGOLIN_LIBS "${PANGOLIN_BUILD}/libpango_*.a")

  target_include_directories(
    _core SYSTEM
    PRIVATE "${ORBSLAM_SRC}"
            "${ORBSLAM_SRC}/include"
            "${ORBSLAM_SRC}/include/CameraModels"
            "${ORBSLAM_SRC}/Thirdparty/Sophus"
            ${PANGOLIN_COMPONENT_INCLUDES}
            "${PANGOLIN_BUILD}/include"
            ${OpenCV_INCLUDE_DIRS})

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
      Boost::serialization
      OpenSSL::Crypto
      GLEW OpenGL GLX GLU EGL SM ICE X11 Xext
      wayland-client wayland-egl wayland-cursor xkbcommon rt)
endif()

# Drop the built module straight into the Python package.
set_target_properties(_core PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/pyorbslam3")
