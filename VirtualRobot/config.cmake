
IF (NOT VIRTUAL_ROBOT_CONFIGURED)

    SET(VIRTUAL_ROBOT_CONFIGURED TRUE)
    
    GET_FILENAME_COMPONENT (CurrentVRPath ${CMAKE_CURRENT_LIST_FILE} PATH)
    SET(VR_DIR ${CurrentVRPath})
    MESSAGE (STATUS "** VR_DIR: ${VR_DIR}")
    SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${VR_DIR}/CMakeModules)
    
    INCLUDE (${VR_DIR}/CMakeModules/VirtualRobotConfigDependencies.cmake)
    
    
     # use virtual folders for grouping projects in IDEs 
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)
    
    ############################# SETUP PATHS #############################

    # Allow #include <VIRTUAL_ROBOT/*.h>
    INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${VR_DIR}/..)
    
 
    # Define, where to put the libs and binaries
    IF (NOT DEFINED BIN_DIR)
        SET(BIN_DIR bin)
    ENDIF()
    IF (NOT DEFINED LIB_DIR)
        SET(LIB_DIR lib)
    ENDIF()
    	
    # check if we are enforced to build to some destination
    IF(DEFINED VIRTUAL_ROBOT_BUILD_DIRECTORY)
    	get_filename_component(VIRTUAL_ROBOT_BUILD_DIRECTORY ${VIRTUAL_ROBOT_BUILD_DIRECTORY} ABSOLUTE)
    	MESSAGE (STATUS "** VIRTUAL_ROBOT Build dir defined: ${VIRTUAL_ROBOT_BUILD_DIRECTORY}")
    ELSE()
    	SET(VIRTUAL_ROBOT_BUILD_DIRECTORY ${CMAKE_BINARY_DIR})
    	MESSAGE (STATUS "** VIRTUAL_ROBOT Build dir not defined, using CMAKE_BINARY_DIR: ${VIRTUAL_ROBOT_BUILD_DIRECTORY}")
    ENDIF()
    
    SET(VIRTUAL_ROBOT_LIB_DIR ${VIRTUAL_ROBOT_BUILD_DIRECTORY}/${LIB_DIR})
    SET(VIRTUAL_ROBOT_BIN_DIR ${VIRTUAL_ROBOT_BUILD_DIRECTORY}/${BIN_DIR})
    
    MESSAGE (STATUS "** VIRTUAL_ROBOT LIB DIR: ${VIRTUAL_ROBOT_LIB_DIR}")
    MESSAGE (STATUS "** VIRTUAL_ROBOT BIN DIR: ${VIRTUAL_ROBOT_BIN_DIR}")
    
    
    # Define, where to install the binaries
    IF (NOT DEFINED VIRTUAL_ROBOT_INSTALL_LIB_DIR)
        SET(VIRTUAL_ROBOT_INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/${LIB_DIR})
    ENDIF()
    IF (NOT DEFINED VIRTUAL_ROBOT_INSTALL_BIN_DIR)
        SET(VIRTUAL_ROBOT_INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/${BIN_DIR})
    ENDIF()
    IF (NOT DEFINED VIRTUAL_ROBOT_INSTALL_HEADER_DIR)
        SET(VIRTUAL_ROBOT_INSTALL_HEADER_DIR ${CMAKE_INSTALL_PREFIX}/include)
    ENDIF()
    
    
    ############################# Set OS specific options #############################
    IF(UNIX)
    	# We are on Linux
    	SET(VIRTUAL_ROBOT_TEST_DIR ${VIRTUAL_ROBOT_BIN_DIR}/tests)
    	IF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
			ADD_DEFINITIONS(-fPIC)
		ENDIF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
	
		IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
			MESSAGE(STATUS "Configuring Debug build")
			ADD_DEFINITIONS(-D_DEBUG) # -Wall -W -Werror -pedantic)
		ELSE()
			MESSAGE(STATUS "Configuring Release build")
		ENDIF()
    ELSE(UNIX)
    	# We are on Windows
    	SET(VIRTUAL_ROBOT_TEST_DIR ${VIRTUAL_ROBOT_BIN_DIR})
    	ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)
		
		# On MSVC we compile with /MP flag (use multiple threads)
		IF(MSVC)
			ADD_DEFINITIONS(/MP)
		ENDIF(MSVC)
    ENDIF(UNIX)
    
    
    #######################################################################
    # Setup for testing
    #######################################################################
    ENABLE_TESTING()
    INCLUDE(CTest)
    
    MESSAGE(STATUS "** Test output directory: ${VIRTUAL_ROBOT_TEST_DIR}")
    
    MACRO(ADD_VR_TEST TEST_NAME)
        
        INCLUDE_DIRECTORIES(${VIRTUAL_ROBOT_EXTERNAL_INCLUDE_DIRS})
        ADD_DEFINITIONS(${VIRTUAL_ROBOT_EXTERNAL_LIBRARY_FLAGS})
    	ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
    	TARGET_LINK_LIBRARIES(${TEST_NAME} VirtualRobot ${VIRTUAL_ROBOT_EXTERNAL_LIBRARIES})
    	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${VIRTUAL_ROBOT_TEST_DIR})
    	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "VirtualRobot Tests")
    	ADD_TEST(NAME VirtualRobot_${TEST_NAME}
    	         COMMAND ${VIRTUAL_ROBOT_TEST_DIR}/${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
    ENDMACRO(ADD_VR_TEST)
    
ENDIF (NOT VIRTUAL_ROBOT_CONFIGURED)
