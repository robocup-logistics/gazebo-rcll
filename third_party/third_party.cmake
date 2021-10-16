include(FetchContent)
find_package(spdlog QUIET)
if (spdlog_FOUND)
  message(STATUS "Found spdlog on system")
else()
  message(STATUS "Fetching spdlog")
  FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_SHALLOW TRUE
    GIT_TAG v1.x
  )
  set(SPDLOG_BUILD_SHARED ON)
  FetchContent_MakeAvailable(spdlog)
endif()
find_package(FreeOpcUa QUIET)
if (FreeOpcUa_FOUND)
  message(STATUS "Found FreeOpcUa on system")
else()
  message(STATUS "Fetching freeopcua")
  FetchContent_Declare(
    FreeOpcUa
    GIT_REPOSITORY https://github.com/FreeOpcUa/freeopcua.git
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(FreeOpcUa)
endif()
