
# Build and helper macros

function(VirtualRobotQtApplication name srcs incs mocFiles uiFiles)

    MESSAGE (STATUS "Qt Moc'ing: ${mocFiles}")
    qt4_wrap_cpp(generatedMocFiles ${mocFiles})
    MESSAGE (STATUS "Qt ui files: ${uiFiles}")
    qt4_wrap_ui(generatedUiFiles ${uiFiles})
    INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )

    ################################## EXECUTABLE ##############################
    ADD_EXECUTABLE(${name} ${srcs} ${incs} ${generatedUiFiles} ${generatedMocFiles})
    TARGET_LINK_LIBRARIES(${name} ${VIRTUAL_ROBOT_LIBRARIES})

endfunction()

function(SimoxQtApplication name srcs incs mocFiles uiFiles)

    VirtualRobotQtApplication("${name}" "${srcs}" "${incs}" "${mocFiles}" "${uiFiles}")
        
    # add Saba and GraspStudio
    TARGET_LINK_LIBRARIES(${name} GraspStudio Saba)
endfunction()
