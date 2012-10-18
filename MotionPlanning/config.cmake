IF (NOT SABA_CONFIGURED)

	# defines SABA_CONFIGURED variable which indicates that this config file has already been included
	SET(SABA_CONFIGURED TRUE)
    
    GET_FILENAME_COMPONENT (CurrentSabaPath ${CMAKE_CURRENT_LIST_FILE} PATH)
    SET(SABA_DIR ${CurrentSabaPath})
    
    ############################# SETUP PATHS TO Simox #############################
    SET(Simox_DIR_STANDARD "${CurrentSabaPath}/..")
    # be sure to have the absolute path
    get_filename_component(Simox_DIR_STANDARD ${Simox_DIR_STANDARD} ABSOLUTE)
    
    SET (SABA_SimoxDir ${Simox_DIR_STANDARD} CACHE STRING "Path to Simox used by SaBa")
    
    INCLUDE(${SABA_SimoxDir}/config.cmake)
    
    IF(NOT DEFINED Simox_BUILD_DIRECTORY)
    	get_filename_component(Simox_BUILD_DIRECTORY ${Simox_BUILD_DIRECTORY} ABSOLUTE)
    	SET(Simox_BUILD_DIRECTORY "${SABA_SimoxDir}/build" CACHE STRING "Simox build directory used by SaBa")
    	SET(Simox_LIB_DIR ${Simox_BUILD_DIRECTORY}/lib)
    	SET(Simox_BIN_DIR ${Simox_BUILD_DIRECTORY}/bin)
    ENDIF()
    
    ############################# SETUP PATHS #############################
    #ADD_DEFINITIONS(-DSABA_BASE_DIR="${SABA_DIR}")
    
    # Define, where to put the binaries
    SET(SABA_LIB_DIR ${Simox_LIB_DIR})
    SET(SABA_BIN_DIR ${Simox_BIN_DIR})
    MESSAGE(STATUS "** SABA_LIB_DIR: ${SABA_LIB_DIR}")
    MESSAGE(STATUS "** SABA_BIN_DIR: ${SABA_BIN_DIR}")
    
    # Define, where to install the binaries
    SET(SABA_INSTALL_LIB_DIR ${Simox_INSTALL_LIB_DIR})
    SET(SABA_INSTALL_BIN_DIR ${Simox_INSTALL_BIN_DIR})
    SET(SABA_INSTALL_HEADER_DIR ${Simox_INSTALL_HEADER_DIR})
    
    #######################################################################
    # Setup for testing
    #######################################################################
    ENABLE_TESTING()
    INCLUDE(CTest)
    
    MACRO(ADD_SABA_TEST TEST_NAME)
    	ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
    	TARGET_LINK_LIBRARIES(${TEST_NAME} VirtualRobot Saba ${VirtualRobot_LINK_LIBRARIES})
    	IF(NOT UNIX)
    	   SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
    	ENDIF(NOT UNIX)
    	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "Saba Tests")
        ADD_TEST(NAME Saba_${TEST_NAME}
    	         COMMAND ${VirtualRobot_TEST_DIR}/${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
    ENDMACRO(ADD_SABA_TEST)
ENDIF(NOT SABA_CONFIGURED)