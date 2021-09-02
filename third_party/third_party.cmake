include(FetchContent)
find_package(FreeOpcUa)
if (FreeOpcUa_FOUND)
  message(STATUS "Found FreeOpcUa on system")
  find_package(spdlog REQUIRED)
else()
  FetchContent_Declare(
    FreeOpcUa
    GIT_REPOSITORY https://github.com/FreeOpcUa/freeopcua.git
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(FreeOpcUa)
endif()
