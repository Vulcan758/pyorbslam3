# Superbuild that fetches and compiles pyorbslam3's native dependencies:
# Pangolin 0.9 (static, PIC, viewer only) and ORB-SLAM3 at a pinned commit with
# the patches in docker/patches/. Driven by the top-level CMakeLists.txt when
# no prebuilt ORB-SLAM3 tree is available (PYORBSLAM3_FETCH_DEPS=ON).
cmake_minimum_required(VERSION 3.16)
project(pyorbslam3_deps NONE)
include(ExternalProject)

set(JOBS 2 CACHE STRING "Parallel compile jobs (ORB-SLAM3 needs ~2 GB RAM per job)")
set(PANGOLIN_VERSION 0.9)
set(ORBSLAM3_COMMIT 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4)
set(PATCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../docker/patches")
set(PANGOLIN_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/pangolin-install")

ExternalProject_Add(pangolin
  URL https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v${PANGOLIN_VERSION}.tar.gz
  URL_HASH SHA256=d3ce8637995b06d5b63238624a3a6b04214a0e96b07bc8e1e8249aa24e52e27b
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF
             -DCMAKE_POSITION_INDEPENDENT_CODE=ON
             -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
             -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_PANGOLIN_VIDEO=OFF
             -DCMAKE_INSTALL_PREFIX=${PANGOLIN_PREFIX}
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -j${JOBS}
  INSTALL_COMMAND ${CMAKE_COMMAND} --install <BINARY_DIR>
)

ExternalProject_Add(orbslam3
  URL https://github.com/UZ-SLAMLab/ORB_SLAM3/archive/${ORBSLAM3_COMMIT}.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  DEPENDS pangolin
  SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/orbslam3-src"
  BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/orbslam3-build"
  PATCH_COMMAND patch -p1 -N -i "${PATCH_DIR}/0001-cmake-static-pic-modern-targets.patch"
        COMMAND patch -p1 -N -i "${PATCH_DIR}/0002-localmapping-keyframeculling-guards.patch"
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
             -DPangolin_DIR=${PANGOLIN_PREFIX}/lib/cmake/Pangolin
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -j${JOBS}
  INSTALL_COMMAND ""
)
