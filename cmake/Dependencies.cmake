include(FetchContent)

# The robot model and algorithms come from the native Pinocchio installation.
# PlaCo is vendored in this repository so experiments can modify its source.
# The remaining small libraries are fetched as pinned source revisions.
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build fetched dependencies as static libraries" FORCE)
set(JSONCPP_WITH_TESTS OFF CACHE BOOL "" FORCE)
set(JSONCPP_WITH_POST_BUILD_UNITTEST OFF CACHE BOOL "" FORCE)
set(JSONCPP_WITH_EXAMPLE OFF CACHE BOOL "" FORCE)
set(JSONCPP_WITH_PKGCONFIG_SUPPORT OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  jsoncpp
  GIT_REPOSITORY https://github.com/open-source-parsers/jsoncpp.git
  GIT_TAG 89e2973c754a9c02a49974d839779b151e95afd6
  GIT_SHALLOW FALSE
  UPDATE_DISCONNECTED TRUE
  EXCLUDE_FROM_ALL
  SYSTEM
)
FetchContent_MakeAvailable(jsoncpp)
if(NOT TARGET jsoncpp_lib AND TARGET jsoncpp_static)
  add_library(jsoncpp_lib ALIAS jsoncpp_static)
endif()

# Official MCAP C++ release `releases/cpp/v2.1.3`, commit
# 1420296ffcfdcde4b6894c0c1aba0ad083f93dde. GitHub's release-tag archive is
# content-checked so configure cannot silently consume a moving branch or
# changed download.
FetchContent_Declare(
  mcap_cpp
  URL https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v2.1.3.tar.gz
  URL_HASH SHA256=181a7c52cc982444cdef533087b321538b025fcc5eec258e0489d68b9ce53ded
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SOURCE_SUBDIR mcl-no-upstream-cmake
  EXCLUDE_FROM_ALL
  SYSTEM
)
FetchContent_MakeAvailable(mcap_cpp)
FetchContent_GetProperties(mcap_cpp SOURCE_DIR MCAP_CPP_SOURCE_DIR)

# MCAP is header-only but its compressed reader needs a concrete zstd target.
# Prefer a native package (MCAP requires >= 1.5.2); otherwise build the pinned
# upstream zstd release in the build tree.
find_package(zstd 1.5.2 CONFIG QUIET)
if(NOT TARGET zstd::libzstd_static AND NOT TARGET zstd::libzstd_shared AND
   NOT TARGET zstd::libzstd)
  set(ZSTD_BUILD_PROGRAMS OFF CACHE BOOL "" FORCE)
  set(ZSTD_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
  set(ZSTD_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(ZSTD_BUILD_SHARED OFF CACHE BOOL "" FORCE)
  set(ZSTD_BUILD_STATIC ON CACHE BOOL "" FORCE)
  FetchContent_Declare(
    zstd_source
    URL https://github.com/facebook/zstd/releases/download/v1.5.7/zstd-1.5.7.tar.gz
    URL_HASH SHA256=eb33e51f49a15e023950cd7825ca74a4a2b43db8354825ac24fc1b7ee09e6fa3
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR build/cmake
    EXCLUDE_FROM_ALL
    SYSTEM
  )
  FetchContent_MakeAvailable(zstd_source)
endif()

if(TARGET zstd::libzstd_static)
  set(MCL_ZSTD_TARGET zstd::libzstd_static)
elseif(TARGET zstd::libzstd_shared)
  set(MCL_ZSTD_TARGET zstd::libzstd_shared)
elseif(TARGET zstd::libzstd)
  set(MCL_ZSTD_TARGET zstd::libzstd)
elseif(TARGET libzstd_static)
  set(MCL_ZSTD_TARGET libzstd_static)
else()
  message(FATAL_ERROR "A zstd target is required for MCAP chunk decoding")
endif()

# Prefer the independently installed shared package required by an installed
# motion_control_core. Standalone PlaCo experiments retain the pinned static
# fallback when no package is available.
find_package(eiquadprog CONFIG QUIET)
if(NOT TARGET eiquadprog::eiquadprog)
  # eiquadprog's upstream CMake layer fetches additional unpinned build modules.
  # Build its two source files directly so the fallback graph remains pinned.
  FetchContent_Declare(
    eiquadprog_source
    GIT_REPOSITORY https://github.com/stack-of-tasks/eiquadprog.git
    GIT_TAG ec402b4dbcce32fd936fd39a3c6fc32f08b35a54
    GIT_SHALLOW FALSE
    UPDATE_DISCONNECTED TRUE
    SOURCE_SUBDIR mcl-no-upstream-cmake
    SYSTEM
  )
  FetchContent_MakeAvailable(eiquadprog_source)
  FetchContent_GetProperties(eiquadprog_source SOURCE_DIR EIQ_SOURCE_DIR)

  add_library(
    eiquadprog
    STATIC
    "${EIQ_SOURCE_DIR}/src/eiquadprog.cpp"
    "${EIQ_SOURCE_DIR}/src/eiquadprog-fast.cpp"
  )
  add_library(eiquadprog::eiquadprog ALIAS eiquadprog)
  target_include_directories(
    eiquadprog
    SYSTEM PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/compat>"
      "$<BUILD_INTERFACE:${EIQ_SOURCE_DIR}/include>"
  )
  target_link_libraries(eiquadprog PUBLIC Eigen3::Eigen)
  target_compile_definitions(eiquadprog PUBLIC EIQUADPROG_STATIC)
  set_target_properties(eiquadprog PROPERTIES POSITION_INDEPENDENT_CODE ON)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    target_compile_options(eiquadprog PRIVATE -Wno-sign-conversion)
  endif()
endif()

set(
  MCL_PLACO_SOURCE_DIR
  "${PROJECT_SOURCE_DIR}/third_party/placo"
  CACHE PATH
  "PlaCo source directory"
)
if(NOT EXISTS "${MCL_PLACO_SOURCE_DIR}/CMakeLists.txt")
  message(
    FATAL_ERROR
    "PlaCo source is missing from MCL_PLACO_SOURCE_DIR=${MCL_PLACO_SOURCE_DIR}"
  )
endif()

set(PLACO_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
add_subdirectory(
  "${MCL_PLACO_SOURCE_DIR}"
  "${CMAKE_CURRENT_BINARY_DIR}/third_party/placo"
  EXCLUDE_FROM_ALL
  SYSTEM
)
