# Macros for Simox executables

function(simox_add_executable PROJECT_NAME SOURCES HEADERS GUI_MOC_HDRS GUI_UIS)

    LINK_DIRECTORIES(${SIMOX_LIB_DIR})
    
    # QT stuff 
    qt4_wrap_cpp(SOURCES ${GUI_MOC_HDRS})
    qt4_wrap_ui(UI_HEADER ${GUI_UIS})
    get_filename_component(UI_HEADER_DIR ${UI_HEADER} PATH)
    list(APPEND HEADERS ${UI_HEADER})
    include_directories(${UI_HEADER_DIR})

    # setup target
    ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCES} ${HEADERS})
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${SIMOX_BIN_DIR})
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} VirtualRobot Saba GraspStudio ${QT_LIBRARIES} ${COIN3D_LIBRARIES} ${SoQt_LIBRARIES})
    INCLUDE_DIRECTORIES(${SoQt_INCLUDE_DIRS})
    INCLUDE(${QT_USE_FILE})
    ADD_DEFINITIONS(-DSOQT_DLL)

    MESSAGE( STATUS ${PROJECT_NAME} " will be placed into " ${SIMOX_BIN_DIR})
endfunction()
