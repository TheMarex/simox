
IF (NOT Simox_CONFIGURED)

	# defines Simox_CONFIGURED variable which indicates that this config file has already been included
	SET(Simox_CONFIGURED TRUE)
		
	# Set up for debug build
	IF(NOT CMAKE_BUILD_TYPE)
	  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
	      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
	      FORCE)
	ENDIF(NOT CMAKE_BUILD_TYPE)
	
	############################# VERSION #################################
	set(Simox_MAJOR_VERSION 2)
	set(Simox_MINOR_VERSION 1)
	set(Simox_PATCH_VERSION 8)
	set(Simox_VERSION
    ${Simox_MAJOR_VERSION}.${Simox_MINOR_VERSION}.${Simox_PATCH_VERSION})

	MESSAGE (STATUS " ** Simox version: ${Simox_VERSION}")
	
	############################# SETUP PATHS #############################
	SET(Simox_BUILD_DIRECTORY ${CMAKE_BINARY_DIR})
	SET(BIN_DIR bin)
	SET(LIB_DIR lib)
	
	SET(Simox_LIB_DIR ${Simox_BUILD_DIRECTORY}/${LIB_DIR})
	SET(Simox_BIN_DIR ${Simox_BUILD_DIRECTORY}/${BIN_DIR})
	
	MESSAGE (STATUS "** SIMOX LIB DIR: ${Simox_LIB_DIR}")
	MESSAGE (STATUS "** SIMOX BIN DIR: ${Simox_BIN_DIR}")
	
	SET(Simox_INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/${LIB_DIR})
	SET(Simox_INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/${BIN_DIR})
	SET(Simox_INSTALL_HEADER_DIR ${CMAKE_INSTALL_PREFIX}/include)
	
	
	## set library settings
	
	    ## SETUP VIRTUAL ROBOT 
    	SET(VirtualRobot_BUILD_DIRECTORY ${Simox_BUILD_DIRECTORY})
    	SET(VirtualRobot_INSTALL_LIB_DIR ${Simox_INSTALL_LIB_DIR})
        SET(VirtualRobot_INSTALL_BIN_DIR ${Simox_INSTALL_BIN_DIR})
        SET(VirtualRobot_INSTALL_HEADER_DIR ${Simox_INSTALL_HEADER_DIR})
        #MESSAGE ("CMAKE_CURRENT_LIST_DIR:${CMAKE_CURRENT_LIST_DIR}")
        include (VirtualRobot/config.cmake)
        # set SIMOX vars according to VirtualRobot config
       	SET(Simox_VISUALIZATION ${VirtualRobot_VISUALIZATION})
		SET(Simox_USE_COIN_VISUALIZATION ${VirtualRobot_USE_COIN_VISUALIZATION})
		SET(Simox_USE_OPENSCENEGRAPH_VISUALIZATION ${VirtualRobot_USE_OPENSCENEGRAPH_VISUALIZATION})
		SET(Simox_VISUALIZATION_LIBS ${VirtualRobot_VISUALIZATION_LIBS})
		SET(Simox_VISUALIZATION_INCLUDE_PATHS ${VirtualRobot_VISUALIZATION_INCLUDE_PATHS})
		SET(Simox_VISUALIZATION_COMPILE_FLAGS ${VirtualRobot_VISUALIZATION_COMPILE_FLAGS})
        
        ## SETUP SABA
        # nothing to do yet

        ## SETUP GRASP STUDIO
        # nothing to do yet

     
ENDIF(NOT Simox_CONFIGURED)
