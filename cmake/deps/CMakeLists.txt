# Superbuild that fetches and compiles pyorbslam3's native dependencies:
# Pangolin 0.9 (static, PIC, viewer only) and ORB-SLAM3 at a pinned commit with
# the patches in docker/patches/. With SELF_CONTAINED=ON it also builds a
# static, minimal OpenCV and installs Eigen, so the resulting extension module
# depends on nothing but libc, libstdc++, OpenGL/X11 and Boost/OpenSSL (which
# auditwheel bundles) — that is how the manylinux wheels are made.
#
# Driven by the top-level CMakeLists.txt when no prebuilt ORB-SLAM3 tree is
# available (PYORBSLAM3_FETCH_DEPS=ON).
cmake_minimum_required(VERSION 3.16)
project(pyorbslam3_deps NONE)
include(ExternalProject)

set(JOBS 2 CACHE STRING "Parallel compile jobs (ORB-SLAM3 needs ~2 GB RAM per job)")
if(JOBS STREQUAL "")
  set(JOBS 2)   # never fall through to an unbounded `cmake --build -j`
endif()
option(SELF_CONTAINED "Also build static OpenCV and install Eigen" OFF)
set(PANGOLIN_VERSION 0.9)
set(ORBSLAM3_COMMIT 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4)
set(OPENCV_VERSION 4.10.0)
set(EIGEN_VERSION 3.4.0)
set(PATCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../docker/patches")
set(PANGOLIN_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/pangolin-install")
set(OPENCV_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/opencv-install")
set(EIGEN_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/eigen-install")

set(_prefix_path "")
set(_pangolin_deps "")
set(_orbslam3_extra_args "")

if(SELF_CONTAINED)
  # ---- Eigen (header-only; installed so Eigen3Config.cmake exists) ----------
  ExternalProject_Add(eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz
    URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EIGEN_PREFIX} -DBUILD_TESTING=OFF
               -DEIGEN_BUILD_DOC=OFF -DEIGEN_BUILD_PKGCONFIG=OFF
    BUILD_COMMAND ""
    INSTALL_COMMAND ${CMAKE_COMMAND} --install <BINARY_DIR>
  )

  # ---- OpenCV: static, only the modules ORB-SLAM3 uses ----------------------
  ExternalProject_Add(opencv
    URL https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.tar.gz
    URL_HASH SHA256=b2171af5be6b26f7a06b1229948bbb2bdaa74fcf5cd097e0af6378fce50a6eb9
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${OPENCV_PREFIX}
               -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
               -DBUILD_LIST=core,imgproc,imgcodecs,features2d,calib3d,highgui
               -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF
               -DBUILD_opencv_apps=OFF -DBUILD_JAVA=OFF -DBUILD_opencv_python3=OFF
               -DBUILD_opencv_python2=OFF -DBUILD_DOCS=OFF
               -DWITH_GTK=OFF -DWITH_QT=OFF -DWITH_WAYLAND=OFF -DWITH_OPENGL=OFF
               -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF -DWITH_V4L=OFF -DWITH_1394=OFF
               -DWITH_OPENEXR=OFF -DWITH_TIFF=OFF -DWITH_WEBP=OFF -DWITH_OPENJPEG=OFF
               -DWITH_JASPER=OFF -DWITH_IPP=OFF -DWITH_ITT=OFF -DWITH_OPENCL=OFF
               -DWITH_CUDA=OFF -DWITH_TBB=OFF -DWITH_EIGEN=OFF -DWITH_PROTOBUF=OFF
               -DWITH_ADE=OFF -DWITH_LAPACK=OFF -DWITH_VA=OFF -DWITH_VA_INTEL=OFF
               -DBUILD_ZLIB=ON -DBUILD_PNG=ON -DBUILD_JPEG=ON
               -DOPENCV_GENERATE_PKGCONFIG=OFF -DENABLE_PRECOMPILED_HEADERS=OFF
    BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -j${JOBS}
    INSTALL_COMMAND ${CMAKE_COMMAND} --install <BINARY_DIR>
  )
  set(_prefix_path "${EIGEN_PREFIX}")
  set(_pangolin_deps eigen)
  # lib vs lib64 differs between distributions: let find_package search the prefixes
  set(_orbslam3_extra_args "-DCMAKE_PREFIX_PATH=${OPENCV_PREFIX}|${EIGEN_PREFIX}")
endif()

ExternalProject_Add(pangolin
  URL https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v${PANGOLIN_VERSION}.tar.gz
  URL_HASH SHA256=d3ce8637995b06d5b63238624a3a6b04214a0e96b07bc8e1e8249aa24e52e27b
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  DEPENDS ${_pangolin_deps}
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF
             -DCMAKE_POSITION_INDEPENDENT_CODE=ON
             -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
             -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_PANGOLIN_VIDEO=OFF
             -DCMAKE_PREFIX_PATH=${_prefix_path}
             -DCMAKE_INSTALL_PREFIX=${PANGOLIN_PREFIX}
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -j${JOBS}
  INSTALL_COMMAND ${CMAKE_COMMAND} --install <BINARY_DIR>
)

set(_orbslam3_deps pangolin)
if(SELF_CONTAINED)
  list(APPEND _orbslam3_deps opencv eigen)
endif()

ExternalProject_Add(orbslam3
  URL https://github.com/UZ-SLAMLab/ORB_SLAM3/archive/${ORBSLAM3_COMMIT}.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  DEPENDS ${_orbslam3_deps}
  SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/orbslam3-src"
  BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/orbslam3-build"
  PATCH_COMMAND patch -p1 -N -i "${PATCH_DIR}/0001-cmake-static-pic-modern-targets.patch"
        COMMAND patch -p1 -N -i "${PATCH_DIR}/0002-localmapping-keyframeculling-guards.patch"
  LIST_SEPARATOR |
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
             -DPangolin_DIR=${PANGOLIN_PREFIX}/lib/cmake/Pangolin
             ${_orbslam3_extra_args}
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -j${JOBS}
  INSTALL_COMMAND ""
)
