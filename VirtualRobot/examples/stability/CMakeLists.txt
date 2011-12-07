PROJECT ( StabilityDemo )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)

IF(NOT DEFINED VR_CONFIGURED)
	INCLUDE(../../config.cmake)
ENDIF()

IF(SIMOX_USE_COIN_VISUALIZATION)
    LINK_DIRECTORIES(${SIMOX_LIB_DIR})
    
	    
	# the variable "demo_SRCS" contains all .cpp files of this project
	FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/stabilityDemo.cpp ${PROJECT_SOURCE_DIR}/stabilityWindow.cpp)
	FILE(GLOB demo_INCS ${PROJECT_SOURCE_DIR}/stabilityWindow.h)

  
################################## moc'ing ##############################
    set(GUI_MOC_HDRS
        ${PROJECT_SOURCE_DIR}/stabilityWindow.h
    )

    set(GUI_UI1
        ${PROJECT_SOURCE_DIR}/stabilityScene.ui 
    )
    

    qt4_wrap_cpp(demo_SRCS ${GUI_MOC_HDRS})
    qt4_wrap_ui(UI_HEADER1 ${GUI_UI1})
    get_filename_component(UI_HEADER_DIR1 ${UI_HEADER1} PATH)
    list(APPEND demo_INCS ${UI_HEADER1})

    include_directories(${UI_HEADER_DIR1})


    ADD_EXECUTABLE(${PROJECT_NAME} ${demo_SRCS} ${demo_INCS})
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${SIMOX_BIN_DIR})

    TARGET_LINK_LIBRARIES(${PROJECT_NAME} VirtualRobot ${QT_LIBRARIES} ${COIN3D_LIBRARIES} ${SoQt_LIBRARIES})
    INCLUDE_DIRECTORIES(${SoQt_INCLUDE_DIRS})
    INCLUDE(${QT_USE_FILE})
    ADD_DEFINITIONS(-DSOQT_DLL)

    MESSAGE( STATUS ${PROJECT_NAME} " will be placed into " ${SIMOX_BIN_DIR})
ENDIF(SIMOX_USE_COIN_VISUALIZATION)
