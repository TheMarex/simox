PROJECT ( RrtGui )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)


INCLUDE(${SABA_SimoxDir}/CMakeModules/SimoxProject.cmake)

IF(SIMOX_VISUALIZATION AND SIMOX_USE_COIN_VISUALIZATION)
     
	# the variable "demo_SRCS" contains all .cpp files of this project
	FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/RrtGui.cpp ${PROJECT_SOURCE_DIR}/RrtGuiWindow.cpp)
	FILE(GLOB demo_INCS ${PROJECT_SOURCE_DIR}/RrtGuiWindow.h)
	
    set(GUI_MOC_HDRS
        ${PROJECT_SOURCE_DIR}/RrtGuiWindow.h
    )
    set(GUI_UIS
        ${PROJECT_SOURCE_DIR}/RrtGui.ui
    )


    # create the executable
    simox_add_example(${PROJECT_NAME} "${demo_SRCS}" "${demo_INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
    
    # we need the GraspStudio lib
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} GraspStudio)    
ENDIF()
