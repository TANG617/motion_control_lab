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
