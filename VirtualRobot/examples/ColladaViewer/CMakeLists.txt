PROJECT ( ColladaViewer )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)

IF(VirtualRobot_USE_COIN_VISUALIZATION AND VirtualRobot_USE_COLLADA)


    GET_FILENAME_COMPONENT (CurrentPath ${CMAKE_CURRENT_LIST_FILE} PATH)

    SET(DEMO_BASE_DIR ${PROJECT_SOURCE_DIR})
    ADD_DEFINITIONS(-DDEMO_BASE_DIR="${DEMO_BASE_DIR}")

	FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/RobotViewer.cpp
	                    ${PROJECT_SOURCE_DIR}/showRobotWindow.cpp
	                    ${PROJECT_SOURCE_DIR}/COLLADA/ColladaIO.cpp
	                    ${PROJECT_SOURCE_DIR}/../COLLADA/ColladaParser.cpp)

	FILE(GLOB demo_INCS ${PROJECT_SOURCE_DIR}/showRobotWindow.h
	                    ${PROJECT_SOURCE_DIR}/COLLADA/ColladaIO.h
	                    ${PROJECT_SOURCE_DIR}/../COLLADA/ColladaParser.h
	                    ${PROJECT_SOURCE_DIR}/../COLLADA/ColladaIterators.h)



    ################################## moc'ing ##############################
    set(mocFiles
        ${PROJECT_SOURCE_DIR}/showRobotWindow.h
    )

    set(uiFiles
        ${PROJECT_SOURCE_DIR}/RobotViewer.ui
    )

    qt4_wrap_cpp(demo_SRCS ${mocFiles})
    qt4_wrap_ui(UI_HEADER ${uiFiles})
    get_filename_component(UI_HEADER_DIR ${UI_HEADER} PATH)
    list(APPEND demo_INCS ${UI_HEADER})

    include_directories(${UI_HEADER_DIR})

    add_definitions( -DDEMO_PATH=\"${PROJECT_SOURCE_DIR}\" )
    add_definitions( " -DDOM_DYNAMIC " )
    # create the executable
    ADD_EXECUTABLE(${PROJECT_NAME} ${demo_SRCS} ${demo_INCS})

    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${VirtualRobot_BIN_DIR})

    TARGET_LINK_LIBRARIES(${PROJECT_NAME} VirtualRobot ${VirtualRobot_VISUALIZATION_LIBS} ${VirtualRobot_EXTERNAL_LIBRARIES})

    INCLUDE_DIRECTORIES(${VirtualRobot_VISUALIZATION_INCLUDE_PATHS})
    INCLUDE(${QT_USE_FILE})

    ADD_DEFINITIONS(${VirtualRobot_VISUALIZATION_COMPILE_FLAGS})

    INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION ${VirtualRobot_INSTALL_BIN_DIR}
    COMPONENT Applications
    )

    MESSAGE( STATUS ${PROJECT_NAME} " will be placed into " ${VirtualRobot_BIN_DIR})
    MESSAGE( STATUS ${PROJECT_NAME} " will be installed into " ${VirtualRobot_INSTALL_BIN_DIR})

ENDIF()
