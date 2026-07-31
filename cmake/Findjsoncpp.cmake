if(TARGET jsoncpp_lib)
  set(jsoncpp_FOUND TRUE)
else()
  set(jsoncpp_FOUND FALSE)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(jsoncpp REQUIRED_VARS jsoncpp_FOUND)

