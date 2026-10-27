cmake_minimum_required(VERSION 3.16)
project(pyorbslam3 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Where does ORB-SLAM3 come from? Three options, tried in this order:
#
#  (a) ORBSLAM3_ROOT (+ optional ORBSLAM3_BUILD_DIR, default <root>/build):
#      a standalone ORB_SLAM3 source tree already built as static libraries,
#      with Pangolin discoverable through find_package(Pangolin) / Pangolin_DIR.
#      Also read from the environment, so `ORBSLAM3_ROOT=... pip install .` works.
#  (b) ORBSLAM_DEPS: legacy — the _deps dir of a PRBonn ORB-SLAM-Python build.
#  (c) PYORBSLAM3_FETCH_DEPS=ON (automatic when neither of the above exists):
#      download and compile Pangolin + ORB-SLAM3 (cmake/deps). Slow (30–60 min)
#      but needs nothing beyond the system packages listed in the README.
# ---------------------------------------------------------------------------
set(ORBSLAM3_ROOT "$ENV{ORBSLAM3_ROOT}" CACHE PATH "Standalone ORB_SLAM3 source tree")
set(ORBSLAM3_BUILD_DIR "" CACHE PATH "Build dir of ORBSLAM3_ROOT (default <root>/build)")
set(ORBSLAM_DEPS
    "${CMAKE_CURRENT_SOURCE_DIR}/../ORB-SLAM-Python/build/cp310-cp310-linux_x86_64/_deps"
    CACHE PATH "Legacy: _deps dir of an ORB-SLAM-Python build")
option(PYORBSLAM3_FETCH_DEPS "Download and build Pangolin + ORB-SLAM3" OFF)
# Self-contained: also build a static minimal OpenCV and Eigen, so the module
# does not depend on the system OpenCV (used for the manylinux wheels).
if(DEFINED ENV{PYORBSLAM3_SELF_CONTAINED} AND NOT "$ENV{PYORBSLAM3_SELF_CONTAINED}" STREQUAL "" AND NOT "$ENV{PYORBSLAM3_SELF_CONTAINED}" STREQUAL "0")
  set(_default_self_contained ON)
else()
  set(_default_self_contained OFF)
endif()
option(PYORBSLAM3_SELF_CONTAINED "Bundle OpenCV + Eigen in the dependency build" ${_default_self_contained})
if(PYORBSLAM3_SELF_CONTAINED)
  set(PYORBSLAM3_FETCH_DEPS ON)
endif()
# Where the dependency build lives (shared between Python versions in CI).
set(PYORBSLAM3_DEPS_DIR "$ENV{PYORBSLAM3_DEPS_DIR}" CACHE PATH "Directory for the dependency superbuild")
if(DEFINED ENV{PYORBSLAM3_JOBS} AND NOT "$ENV{PYORBSLAM3_JOBS}" STREQUAL "")
  set(_default_jobs "$ENV{PYORBSLAM3_JOBS}")
else()
  set(_default_jobs 2)
endif()
set(PYORBSLAM3_JOBS "${_default_jobs}" CACHE STRING "Parallel jobs for the dependency build")
if(PYORBSLAM3_JOBS STREQUAL "")
  set(PYORBSLAM3_JOBS 2)
endif()

if(ORBSLAM3_ROOT STREQUAL "" AND NOT PYORBSLAM3_FETCH_DEPS
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3/build/libORB_SLAM3.a")
  set(ORBSLAM3_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3")
endif()
if(ORBSLAM3_ROOT STREQUAL "" AND NOT PYORBSLAM3_FETCH_DEPS
   AND NOT EXISTS "${ORBSLAM_DEPS}/orb_slam3-build/libORB_SLAM3.a")
  set(PYORBSLAM3_FETCH_DEPS ON)
endif()

if(PYORBSLAM3_FETCH_DEPS)
  if(PYORBSLAM3_DEPS_DIR STREQUAL "")
    set(_deps "${CMAKE_BINARY_DIR}/deps")
  else()
    set(_deps "${PYORBSLAM3_DEPS_DIR}")
  endif()
  message(STATUS "pyorbslam3: building Pangolin + ORB-SLAM3 from source in ${_deps} "
                 "(jobs=${PYORBSLAM3_JOBS}, self-contained=${PYORBSLAM3_SELF_CONTAINED}; this takes a while)")
  # Generator, make program and compilers are passed explicitly so that a
  # dependency build cached from an earlier pip run (whose temporary build
  # environment is gone) is refreshed instead of pointing at stale paths.
  execute_process(
    COMMAND ${CMAKE_COMMAND} -S "${CMAKE_CURRENT_SOURCE_DIR}/cmake/deps" -B "${_deps}"
            -G "${CMAKE_GENERATOR}" "-DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}"
            "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}" "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
            -DJOBS=${PYORBSLAM3_JOBS} -DSELF_CONTAINED=${PYORBSLAM3_SELF_CONTAINED}
    RESULT_VARIABLE _rc)
  if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "configuring the dependency superbuild failed")
  endif()
  execute_process(COMMAND ${CMAKE_COMMAND} --build "${_deps}" RESULT_VARIABLE _rc)
  if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "building Pangolin / ORB-SLAM3 failed (see output above)")
  endif()
  set(ORBSLAM3_ROOT "${_deps}/orbslam3-src")
  set(ORBSLAM3_BUILD_DIR "${_deps}/orbslam3-build")
  set(Pangolin_DIR "${_deps}/pangolin-install/lib/cmake/Pangolin" CACHE PATH "" FORCE)
  if(PYORBSLAM3_SELF_CONTAINED)
    # prepend so the bundled OpenCV/Eigen win over any system copies
    list(PREPEND CMAKE_PREFIX_PATH "${_deps}/opencv-install" "${_deps}/eigen-install")
  endif()
endif()

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 REQUIRED)
# Boost.Serialization is linked shared on purpose: distribution static builds
# are usually not -fPIC (manylinux's is not), and auditwheel bundles the .so.
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)

pybind11_add_module(_core MODULE src/orbslam3_pybind.cpp)

if(PYORBSLAM3_SELF_CONTAINED)
  set(_win_uri "x11")   # manylinux builds have no Wayland
else()
  set(_win_uri "wayland")
endif()
target_compile_definitions(
  _core PRIVATE HAVE_GLEW HAVE_EIGEN _LINUX_ PANGO_DEFAULT_WIN_URI="${_win_uri}"
                BOOST_ALL_NO_LIB BOOST_SERIALIZATION_DYN_LINK)

if(NOT ORBSLAM3_ROOT STREQUAL "")
  # ---- (a)/(c) standalone tree ---------------------------------------------
  if(ORBSLAM3_BUILD_DIR STREQUAL "")
    set(ORBSLAM3_BUILD_DIR "${ORBSLAM3_ROOT}/build")
  endif()
  if(NOT EXISTS "${ORBSLAM3_BUILD_DIR}/libORB_SLAM3.a")
    message(FATAL_ERROR "libORB_SLAM3.a not found in ${ORBSLAM3_BUILD_DIR}; build ORB_SLAM3 first.")
  endif()
  find_package(Pangolin REQUIRED)
  message(STATUS "pyorbslam3: linking ORB_SLAM3 from ${ORBSLAM3_ROOT} (build ${ORBSLAM3_BUILD_DIR})")

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
      "${ORBSLAM3_BUILD_DIR}/libORB_SLAM3.a"
      "${ORBSLAM3_BUILD_DIR}/Thirdparty/DBoW2/libDBoW2.a"
      "${ORBSLAM3_BUILD_DIR}/Thirdparty/g2o/libg2o.a"
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

if(SKBUILD)
  # pip / scikit-build-core: the module goes into the wheel's package dir.
  install(TARGETS _core LIBRARY DESTINATION pyorbslam3)
else()
  # ./build.sh: drop the built module straight into the source package.
  set_target_properties(_core PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/pyorbslam3")
endif()
