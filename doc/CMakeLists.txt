
find_package(Doxygen)

if (DOXYGEN_FOUND)
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # set(PACKAGE_VERSION ${OT_PACKAGE_VERSION})
    set(abs_srcdir ${CMAKE_CURRENT_SOURCE_DIR})
    # set(abs_builddir ${CMAKE_CURRENT_BINARY_DIR})
    set(abs_top_srcdir ${PROJECT_SOURCE_DIR})
    # set(abs_top_builddir ${PROJECT_BINARY_DIR})
    # set(DOXYGEN_USE_DOT ${DOXYGEN_HAVE_DOT})

    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    add_custom_target(nmsdk2-doc)

    add_custom_target(nmsdk2-doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        # COMMAND cmake -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/images ${CMAKE_CURRENT_BINARY_DIR}/html/doc/images
        # WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating documentation with Doxygen"
        VERBATIM)
    add_dependencies(nmsdk2-doc nmsdk2-doxygen)

else()
    message("Doxygen must be installed to generate doxygen documentation")
endif()
