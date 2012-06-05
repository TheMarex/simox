PROJECT ( IKRRT )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)

IF(Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)
    LINK_DIRECTORIES(${Simox_LIB_DIR})
    
	    
	# the variable "demo_SRCS" contains all .cpp files of this project
	FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/IKRRTDemo.cpp ${PROJECT_SOURCE_DIR}/IKRRTWindow.cpp)
	FILE(GLOB demo_INCS ${PROJECT_SOURCE_DIR}/IKRRTWindow.h)

  
    ################################## moc'ing ##############################
    set(GUI_MOC_HDRS
        ${PROJECT_SOURCE_DIR}/IKRRTWindow.h
    )

    set(GUI_UIS
        ${PROJECT_SOURCE_DIR}/IKRRT.ui
    )

    qt4_wrap_cpp(demo_SRCS ${GUI_MOC_HDRS})
    qt4_wrap_ui(UI_HEADER ${GUI_UIS})
    get_filename_component(UI_HEADER_DIR ${UI_HEADER} PATH)
    list(APPEND demo_INCS ${UI_HEADER})

    include_directories(${UI_HEADER_DIR})


    ADD_EXECUTABLE(${PROJECT_NAME} ${demo_SRCS} ${demo_INCS})
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})

    TARGET_LINK_LIBRARIES(${PROJECT_NAME} VirtualRobot Saba ${Simox_VISUALIZATION_LIBS})
    INCLUDE_DIRECTORIES(${Simox_VISUALIZATION_INCLUDE_PATHS})
    INCLUDE(${QT_USE_FILE})
    ADD_DEFINITIONS(${Simox_VISUALIZATION_COMPILE_FLAGS})

    INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION ${SABA_INSTALL_BIN_DIR})
        
    MESSAGE( STATUS ${PROJECT_NAME} " will be placed into " ${Simox_BIN_DIR})
    MESSAGE( STATUS ${PROJECT_NAME} " will be installed into " ${SABA_INSTALL_BIN_DIR})
ENDIF()
