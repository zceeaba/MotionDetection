find_package(Doxygen
        REQUIRED dot)

if (DOXYGEN_FOUND)
    set(DOXYGEN_IN Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    add_custom_target(MotionDetectionDocs
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM
            )

    # Install the project's HTML documentation
    install(DIRECTORY DESTINATION ${CMAKE_BINARY_DIR}/doc/html )
else (DOXYGEN_FOUND)
    message(FATAL_ERROR "Doxygen needs to be installed to generate documentation!")
endif (DOXYGEN_FOUND)