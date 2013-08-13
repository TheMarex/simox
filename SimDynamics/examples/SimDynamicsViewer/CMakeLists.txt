PROJECT ( SimDynamicsViewer )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)

IF(Simox_USE_COIN_VISUALIZATION)

    INCLUDE(${QT_USE_FILE})
    
    LINK_DIRECTORIES(${SimDynamics_LIB_DIR})
    
    FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/SimDynamicsViewer.cpp ${PROJECT_SOURCE_DIR}/simDynamicsWindow.cpp)
    FILE(GLOB demo_INCS ${PROJECT_SOURCE_DIR}/simDynamicsWindow.h)

    set(GUI_MOC_HDRS
        ${PROJECT_SOURCE_DIR}/simDynamicsWindow.h
    )

    set(GUI_UIS
        ${PROJECT_SOURCE_DIR}/simDynamicsViewer.ui
    )

    SimoxQtApplication("${PROJECT_NAME}" "${demo_SRCS}" "${demo_INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")

    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${SimDynamics_BIN_DIR})

    TARGET_LINK_LIBRARIES(${PROJECT_NAME} VirtualRobot ${VirtualRobot_VISUALIZATION_LIBS} SimDynamics ${SimDynamics_PHYSICS_LIBRARIES})
    INCLUDE_DIRECTORIES(${VirtualRobot_VISUALIZATION_INCLUDE_PATHS})
    
    INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION ${SimDynamics_INSTALL_BIN_DIR})
        
    MESSAGE( STATUS ${PROJECT_NAME} " will be placed into " ${SimDynamics_BIN_DIR})
    MESSAGE( STATUS ${PROJECT_NAME} " will be installed into " ${SimDynamics_INSTALL_BIN_DIR})
ENDIF()
