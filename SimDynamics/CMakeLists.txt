PROJECT ( SimDynamics )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
CMAKE_POLICY(VERSION 2.6)

MESSAGE (STATUS "\n ***** CONFIGURING Simox project SimDynamics *****")
INCLUDE (config.cmake)

if (SimDynamics_DYNAMICSENGINE)

    SET(SOURCES
    DynamicsWorld.cpp
	DynamicsEngine/DynamicsObject.cpp
	DynamicsEngine/DynamicsEngine.cpp
	DynamicsEngine/DynamicsRobot.cpp
    )
    
    SET(INCLUDES
    DynamicsWorld.h
    SimDynamics.h
	DynamicsEngine/DynamicsEngineFactory.h
	DynamicsEngine/DynamicsObject.h
	DynamicsEngine/DynamicsEngine.h
	DynamicsEngine/DynamicsRobot.h
    ${SimDynamics_SimoxDir}/VirtualRobot/definesVR.h
    )
    
    if (SimDynamics_USE_BULLET)
    	SET(SOURCES
    	${SOURCES}
    	DynamicsEngine/BulletEngine/BulletEngineFactory.cpp
    	DynamicsEngine/BulletEngine/BulletEngine.cpp
    	DynamicsEngine/BulletEngine/BulletObject.cpp
    	DynamicsEngine/BulletEngine/BulletRobot.cpp
    	DynamicsEngine/BulletEngine/SimoxMotionState.cpp
    	)
    
    	SET(INCLUDES
    	${INCLUDES}
    	DynamicsEngine/BulletEngine/BulletEngineFactory.h
    	DynamicsEngine/BulletEngine/BulletEngine.h
    	DynamicsEngine/BulletEngine/BulletObject.h
    	DynamicsEngine/BulletEngine/BulletRobot.h
    	DynamicsEngine/BulletEngine/SimoxMotionState.h
    	)
    	
    	if (SimDynamics_BULLET_OpenGL)
        	SET(SOURCES
        	${SOURCES}
        	DynamicsEngine/BulletEngine/BulletOpenGLViewer.cpp
        	)
        
        	SET(INCLUDES
        	${INCLUDES}
        	DynamicsEngine/BulletEngine/BulletOpenGLViewer.h
        	)
        else()
            MESSAGE (STATUS "Bullet's OpenGLSupport library not found: Skipping BulletOpenGLViewer")
        endif()

		if (Simox_USE_COIN_VISUALIZATION)
        	SET(SOURCES
        	${SOURCES}
        	DynamicsEngine/BulletEngine/BulletCoinQtViewer.cpp
        	)
        
        	SET(INCLUDES
        	${INCLUDES}
        	DynamicsEngine/BulletEngine/BulletCoinQtViewer.h
        	)
        else()
            MESSAGE (STATUS "Qt/Coin3D disabled: Skipping BulletCoinQtViewer")
        endif()
    endif ()
    
    # this var is considered for generating Simox_INCLUDE_DIRS_
    SET(SimDynamics_INCLUDE_DIRS "${SimDynamics_SimoxDir}" "${SimDynamics_DIR}")
    
    INCLUDE_DIRECTORIES(${SimDynamics_SimoxDir})
    INCLUDE_DIRECTORIES(${SimDynamics_DIR})
    
    INCLUDE_DIRECTORIES(${VirtualRobot_EXTERNAL_INCLUDE_DIRS})
    ADD_DEFINITIONS(${VirtualRobot_EXTERNAL_LIBRARY_FLAGS})
    
    
    ADD_LIBRARY (SimDynamics SHARED ${SOURCES} ${INCLUDES})
    TARGET_LINK_LIBRARIES (SimDynamics ${COLLISIONDETECTION_LIB} ${VirtualRobot_EXTERNAL_LIBRARIES} VirtualRobot ${SimDynamics_PHYSICS_LIBRARIES})
    
    # include examples
    ADD_SUBDIRECTORY(examples/)
    
    # include unit tests
    #ADD_SUBDIRECTORY(tests/)
    if (SimDynamics_USE_BULLET)
        ADD_SUBDIRECTORY(DynamicsEngine/BulletEngine/tests/)
    endif()
    
    # .DLL path
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${SimDynamics_BIN_DIR})
    # .so path
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${SimDynamics_LIB_DIR})
    # .lib path (this is needed for setting the DLL-import library path on windows)
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${SimDynamics_LIB_DIR})
    
    INSTALL(TARGETS SimDynamics
      RUNTIME DESTINATION ${SimDynamics_INSTALL_BIN_DIR}
      LIBRARY DESTINATION ${SimDynamics_INSTALL_LIB_DIR}
      ARCHIVE DESTINATION ${SimDynamics_INSTALL_LIB_DIR}
      )
    
    INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} DESTINATION ${SimDynamics_INSTALL_HEADER_DIR}
            FILES_MATCHING PATTERN "*.h"
            PATTERN ".svn" EXCLUDE
            PATTERN "CMakeModules" EXCLUDE
            PATTERN "tests" EXCLUDE
            PATTERN "build*" EXCLUDE
            PATTERN "data" EXCLUDE
            PATTERN "examples" EXCLUDE)
else()
    MESSAGE ("Not building SimDynamics, physics engine not found...")
endif()