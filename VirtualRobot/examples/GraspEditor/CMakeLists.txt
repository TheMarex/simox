PROJECT ( GraspEditor )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)

IF(VirtualRobot_VISUALIZATION AND VirtualRobot_USE_COIN_VISUALIZATION)

    LINK_DIRECTORIES(${VirtualRobot_LIB_DIR})
    
	    
	# the variable "demo_SRCS" contains all .cpp files of this project
	FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/GraspEditor.cpp ${PROJECT_SOURCE_DIR}/GraspEditorWindow.cpp)
	FILE(GLOB demo_INCS ${PROJECT_SOURCE_DIR}/GraspEditorWindow.h)

  
    ################################## moc'ing ##############################
    set(GUI_MOC_HDRS
        ${PROJECT_SOURCE_DIR}/GraspEditorWindow.h
    )

    set(GUI_UIS
        ${PROJECT_SOURCE_DIR}/GraspEditor.ui
    )

    qt4_wrap_cpp(demo_SRCS ${GUI_MOC_HDRS})
    qt4_wrap_ui(UI_HEADER ${GUI_UIS})
    get_filename_component(UI_HEADER_DIR ${UI_HEADER} PATH)
    list(APPEND demo_INCS ${UI_HEADER})

    include_directories(${UI_HEADER_DIR})


    ADD_EXECUTABLE(${PROJECT_NAME} ${demo_SRCS} ${demo_INCS})
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${VirtualRobot_BIN_DIR})

    TARGET_LINK_LIBRARIES(${PROJECT_NAME} VirtualRobot ${VirtualRobot_VISUALIZATION_LIBS})
    INCLUDE_DIRECTORIES(${VirtualRobot_VISUALIZATION_INCLUDE_PATHS})
    INCLUDE(${QT_USE_FILE})
    ADD_DEFINITIONS(${VirtualRobot_VISUALIZATION_COMPILE_FLAGS})
     
    INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION ${VirtualRobot_INSTALL_BIN_DIR})
        
    MESSAGE( STATUS ${PROJECT_NAME} " will be placed into " ${VirtualRobot_BIN_DIR})
    MESSAGE( STATUS ${PROJECT_NAME} " will be installed into " ${VirtualRobot_INSTALL_BIN_DIR})

ENDIF()
