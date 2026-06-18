if(NOT DEFINED REPO_ROOT)
    get_filename_component(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

string(REPLACE "\"" "" REPO_ROOT "${REPO_ROOT}")

find_program(DOXYGEN_EXECUTABLE
    NAMES doxygen doxygen.exe
    HINTS
        "C:/Program Files/doxygen/bin"
        "C:/Program Files (x86)/doxygen/bin"
)

if(NOT DOXYGEN_EXECUTABLE)
    message(WARNING "Doxygen was not found in PATH. Skipping code documentation generation. Install Doxygen to generate docs/api/html/index.html during firmware builds.")
    return()
endif()

execute_process(
    COMMAND "${DOXYGEN_EXECUTABLE}" "${REPO_ROOT}/docs/Doxyfile"
    WORKING_DIRECTORY "${REPO_ROOT}"
    RESULT_VARIABLE DOXYGEN_RESULT
)

if(NOT DOXYGEN_RESULT EQUAL 0)
    message(FATAL_ERROR "Doxygen failed with exit code ${DOXYGEN_RESULT}")
endif()
