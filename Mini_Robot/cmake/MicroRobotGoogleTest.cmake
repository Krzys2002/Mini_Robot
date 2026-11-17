include_guard(GLOBAL)
find_package(GTest CONFIG QUIET)
if(NOT GTest_FOUND)
    include(FetchContent)
    # Match the default MSVC runtime used by the application.
    set(gtest_force_shared_crt ON CACHE BOOL "Use shared MSVC runtime" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "Disable GoogleTest installation" FORCE)
    set(BUILD_GMOCK OFF CACHE BOOL "GoogleMock is not used by this project" FORCE)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz
        URL_HASH SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        TIMEOUT 60
        INACTIVITY_TIMEOUT 15
    )
    FetchContent_MakeAvailable(googletest)
endif()
