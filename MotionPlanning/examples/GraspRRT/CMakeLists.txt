PROJECT ( GraspRrtDemo )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)

IF(SIMOX_VISUALIZATION AND SIMOX_USE_COIN_VISUALIZATION)
	    
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
ENDIF()
