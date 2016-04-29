
# Define target name
set (TARGET_NAME RacingGame)

# Search extra source files
file(GLOB MYSOURCES_ProcSky
    "${CMAKE_CURRENT_SOURCE_DIR}/ProcSky/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/ProcSky/*.cpp"
)

# Define source files
define_source_files (EXTRA_H_FILES ${MYSOURCES_ProcSky})

#MESSAGE( STATUS "<_______HIER:         " ${MYSOURCES_ProcSky} )


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


# Setup target with resource copying
if (EMSCRIPTEN)
    # Override the Urho3D default
    math (EXPR EMSCRIPTEN_TOTAL_MEMORY "64 * 1024 * 1024")
    if (URHO3D_TESTING AND URHO3D_TEST_TIMEOUT LESS 15)
        set (URHO3D_TEST_TIMEOUT 15)
    endif ()
endif ()

# Setup target with resource copying
setup_main_executable ()

# Setup test cases
#setup_test ()
