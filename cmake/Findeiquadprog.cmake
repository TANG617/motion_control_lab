if(TARGET eiquadprog::eiquadprog)
  set(eiquadprog_FOUND TRUE)
else()
  set(eiquadprog_FOUND FALSE)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(eiquadprog REQUIRED_VARS eiquadprog_FOUND)

