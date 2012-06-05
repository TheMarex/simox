PROJECT ( GraspRrtDemo )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)

IF(Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)
	    
	# the variable "demo_SRCS" contains all .cpp files of this project
	FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/GraspRrtDemo.cpp ${PROJECT_SOURCE_DIR}/GraspRrtWindow.cpp)
	FILE(GLOB demo_INCS ${PROJECT_SOURCE_DIR}/GraspRrtWindow.h)

  
    ################################## moc'ing ##############################
    set(GUI_MOC_HDRS
        ${PROJECT_SOURCE_DIR}/GraspRrtWindow.h
    )

    set(GUI_UIS
        ${PROJECT_SOURCE_DIR}/GraspRrt.ui
    )

    SimoxQtApplication("${PROJECT_NAME}" "${demo_SRCS}" "${demo_INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")

	SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})

     # install
	 INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION ${SABA_INSTALL_BIN_DIR})
        
    MESSAGE( STATUS ${PROJECT_NAME} " will be placed into " ${Simox_BIN_DIR})
    MESSAGE( STATUS ${PROJECT_NAME} " will be installed into " ${SABA_INSTALL_BIN_DIR})

  
ENDIF()
