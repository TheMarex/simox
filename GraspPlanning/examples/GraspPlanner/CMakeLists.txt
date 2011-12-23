PROJECT ( GraspPlanner )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)

IF(NOT DEFINED SIMOX_CONFIGURED)
	INCLUDE(../../config.cmake)
ENDIF()
	
IF(SIMOX_VISUALIZATION)

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
    simox_add_executable(${PROJECT_NAME} "${demo_SRCS}" "${demo_INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
    
    # we need the GraspStudio lib
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} GraspStudio)    
ENDIF(SIMOX_VISUALIZATION)
