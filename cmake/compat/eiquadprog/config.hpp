#pragma once

// eiquadprog normally generates this header through jrl-cmakemodules. The lab
// builds the pinned source as a static library, so no platform DLL decoration
// is needed.
#define EIQUADPROG_VERSION "1.3.2"
#define EIQUADPROG_MAJOR_VERSION 1
#define EIQUADPROG_MINOR_VERSION 3
#define EIQUADPROG_PATCH_VERSION 2

#define EIQUADPROG_VERSION_AT_LEAST(major, minor, patch)                                      \
  (EIQUADPROG_MAJOR_VERSION > major ||                                                       \
   (EIQUADPROG_MAJOR_VERSION >= major &&                                                     \
    (EIQUADPROG_MINOR_VERSION > minor ||                                                     \
     (EIQUADPROG_MINOR_VERSION >= minor && EIQUADPROG_PATCH_VERSION >= patch))))

#define EIQUADPROG_DLLAPI
#define EIQUADPROG_LOCAL
#define EIQUADPROG_EXPLICIT_INSTANTIATION_DECLARATION extern template
#define EIQUADPROG_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
#define EIQUADPROG_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI

