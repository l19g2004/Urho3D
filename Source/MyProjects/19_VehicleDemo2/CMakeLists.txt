
# Define target name
set (TARGET_NAME 19_VehicleDemo2)

# Define source files
define_source_files (EXTRA_H_FILES ${COMMON_SAMPLE_H_FILES})

# Setup target with resource copying
if (EMSCRIPTEN)
    # Override the Urho3D default
    math (EXPR EMSCRIPTEN_TOTAL_MEMORY "64 * 1024 * 1024")
    if (URHO3D_TESTING AND URHO3D_TEST_TIMEOUT LESS 15)
        set (URHO3D_TEST_TIMEOUT 15)
    endif ()
endif ()
setup_main_executable ()

# Setup test cases
setup_test ()
