PROJECT ( GraspPlanner )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)
	
IF(Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)

	# the variable "demo_SRCS" contains all .cpp files of this project
	FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/GraspPlanner.cpp ${PROJECT_SOURCE_DIR}/GraspPlannerWindow.cpp)
	FILE(GLOB demo_INCS ${PROJECT_SOURCE_DIR}/GraspPlannerWindow.h)
	
    set(GUI_MOC_HDRS
        ${PROJECT_SOURCE_DIR}/GraspPlannerWindow.h
    )
    set(GUI_UIS
        ${PROJECT_SOURCE_DIR}/GraspPlanner.ui
    )
    
    # create the executable
    SimoxQtApplication(${PROJECT_NAME} "${demo_SRCS}" "${demo_INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
    
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${GRASPSTUDIO_BIN_DIR})
    
     # install
    INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION ${GRASPSTUDIO_INSTALL_BIN_DIR})
        
    MESSAGE( STATUS ${PROJECT_NAME} " will be placed into " ${GRASPSTUDIO_BIN_DIR})
    MESSAGE( STATUS ${PROJECT_NAME} " will be installed into " ${GRASPSTUDIO_INSTALL_BIN_DIR})
ENDIF()
