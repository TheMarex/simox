IF (NOT SimDynamics_CONFIGURED)

	# defines SimDynamics_CONFIGURED variable which indicates that this config file has already been included
	SET(SimDynamics_CONFIGURED TRUE)
	

    GET_FILENAME_COMPONENT (CurrentSimDynamicsPath ${CMAKE_CURRENT_LIST_FILE} PATH)
    SET(SimDynamics_DIR ${CurrentSimDynamicsPath})
	SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${SimDynamics_DIR}/CMakeModules)
     
    ############################# SETUP PATHS TO Simox #############################
    SET(SimDynamics_Simox_DIR_STANDARD "${CurrentSimDynamicsPath}/..")
    # be sure to have the absolute path
    get_filename_component(SimDynamics_Simox_DIR_STANDARD ${SimDynamics_Simox_DIR_STANDARD} ABSOLUTE)
    
    SET (SimDynamics_SimoxDir ${SimDynamics_Simox_DIR_STANDARD} CACHE STRING "Path to Simox used by SimDynamics")
    
    INCLUDE(${SimDynamics_SimoxDir}/config.cmake)
    	
    IF(NOT DEFINED Simox_BUILD_DIRECTORY)
     	SET(Simox_BUILD_DIRECTORY "${SimDynamics_SimoxDir}/build" CACHE STRING "Simox build directory used by SimDynamics")
    	SET(Simox_LIB_DIR ${Simox_BUILD_DIRECTORY}/lib)
    	SET(Simox_BIN_DIR ${Simox_BUILD_DIRECTORY}/bin)
    ENDIF()
    
    ############################# SETUP PATHS #############################
     
    # Define, where to put the binaries
    SET(SimDynamics_LIB_DIR ${Simox_LIB_DIR})
    SET(SimDynamics_BIN_DIR ${Simox_BIN_DIR})
    MESSAGE(STATUS "** SimDynamics_LIB_DIR: ${SimDynamics_LIB_DIR}")
    MESSAGE(STATUS "** SimDynamics_BIN_DIR: ${SimDynamics_BIN_DIR}")
    
    # Define, where to install the binaries
    SET(SimDynamics_INSTALL_LIB_DIR ${Simox_INSTALL_LIB_DIR})
    SET(SimDynamics_INSTALL_BIN_DIR ${Simox_INSTALL_BIN_DIR})
    SET(SimDynamics_INSTALL_HEADER_DIR ${Simox_INSTALL_HEADER_DIR})
    
    
    ############################# SETUP PHYSICS ENGINE #############################
   	OPTION(SimDynamics_USE_BULLET "Use Bullet Physics Engine" ON)
   	SET (SimDynamics_DYNAMICSENGINE FALSE)
   	SET (SimDynamics_PHYSICS_LIBRARIES "")
   	
   	if (SimDynamics_USE_BULLET)
   	
       	IF(NOT "$ENV{BULLET_ROOT}" STREQUAL "")
    	    MESSAGE (STATUS "USING BULLET_ROOT-PATH from environment variable BULLET_ROOT: $ENV{BULLET_ROOT}")
    	    file(TO_CMAKE_PATH "$ENV{BULLET_ROOT}" BULLET_ROOT)
    	ENDIF()
    	#SET (SimDynamics_BULLET_ROOT ${Simox_BULLET_ROOT} CACHE PATH "Path to Bullet used by SimDynamics")
   	    SET (BULLET_ROOT ${BULLET_ROOT} CACHE PATH "Bullet Path")
   	    Find_Package(Bullet)
   	    if (BULLET_FOUND)
   	        MESSAGE (STATUS "Found Bullet at ${BULLET_INCLUDE_DIR}")
   	        #MESSAGE (STATUS "BULLET_LIBRARIES: ${BULLET_LIBRARIES}")
   	        #MESSAGE (STATUS "BULLET_OPENGL_INCLUDE_DIR: ${BULLET_OPENGL_INCLUDE_DIR}")
   	        #MESSAGE (STATUS "BULLET_OpenGLSupport_LIBRARY_debug: ${BULLET_OpenGLSupport_LIBRARY_debug}")
           	INCLUDE_DIRECTORIES( 
                ${BULLET_INCLUDE_DIR}
                ${BULLET_INCLUDE_DIR}/bullet
                ${BULLET_DEMOS_INCLUDE_DIR}
                ${BULLET_OPENGL_INCLUDE_DIR}
            )
            OPTION( SimDynamics_USE_BULLET_DOUBLE_PRECISION "Use Bullet Engine built with double precision" OFF )
            OPTION(SimDynamics_USE_BULLET_USE_GLUT "Use Glut"	ON)
            IF( SimDynamics_USE_BULLET_DOUBLE_PRECISION )
                ADD_DEFINITIONS( -DBT_USE_DOUBLE_PRECISION)
            ENDIF( SimDynamics_USE_BULLET_DOUBLE_PRECISION )
            SET (SimDynamics_PHYSICS_LIBRARIES "${BULLET_LIBRARIES}")
            SET (SimDynamics_DYNAMICSENGINE TRUE)
            
            IF (SimDynamics_USE_BULLET_USE_GLUT)
	            FIND_PACKAGE(OpenGL)
	            IF (OPENGL_FOUND)
		      MESSAGE ("OPENGL FOUND lib:"${OPENGL_gl_LIBRARY})
		    ENDIF()
	            IF (OPENGL_GLU_FOUND)
		      MESSAGE ("OPENGL_GLU FOUND lib:"${OPENGL_glu_LIBRARY})
		    ENDIF()

	            FIND_PACKAGE(GLUT)
	            IF (GLUT_FOUND)
	                MESSAGE("GLUT FOUND")
	                MESSAGE(${GLUT_glut_LIBRARY})
	                SET(SimDynamics_PHYSICS_LIBRARIES ${SimDynamics_PHYSICS_LIBRARIES} ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
	            ELSE (GLUT_FOUND)
    	            SET( GLUT_glut_LIBRARY "" CACHE PATH "Glut library." )
                    #IF (MSVC)
                    #IF (CMAKE_CL_64)
    	            #    message("win64 using glut64.lib")
    	            #    SET(GLUT_glut_LIBRARY ${BULLET_PHYSICS_SOURCE_DIR}/Glut/glut64.lib)
                    #ELSE(CMAKE_CL_64)
                    #	message("win32 using glut32.lib")
                    #	SET(GLUT_glut_LIBRARY ${BULLET_PHYSICS_SOURCE_DIR}/Glut/glut32.lib)
                    #ENDIF (CMAKE_CL_64)
                	#ENDIF (MSVC)
                ENDIF (GLUT_FOUND)

            	IF (WIN32)
            	    INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR})
            	ELSE (WIN32)
            	    # This is the lines for linux.  This should always work if everything is installed and working fine.
            	    INCLUDE_DIRECTORIES(/usr/include /usr/local/include ${GLUT_INCLUDE_DIR}) 
            	ENDIF (WIN32)

            ENDIF(SimDynamics_USE_BULLET_USE_GLUT)
            
            if (SimDynamics_USE_BULLET_USE_GLUT AND GLUT_glut_LIBRARY AND DEFINED BULLET_OpenGLSupport_LIBRARY)
                MESSAGE ("Enabling OpenGL / Glut support")
                SET (SimDynamics_BULLET_OpenGL TRUE)
            else()
                MESSAGE ("Disabling OpenGL / Glut support")
                SET (SimDynamics_BULLET_OpenGL FALSE)
            endif()
                
        else()
            MESSAGE ("Could not find Bullet")
        endif()
        
   	else()
            MESSAGE ("No Physics engine selected...")   	    
   	endif()
   	

    
    #######################################################################
    # Setup for testing
    #######################################################################
    ENABLE_TESTING()
    INCLUDE(CTest)
    
    MACRO(ADD_SIMDYNAMICS_TEST TEST_NAME)
    	ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
    	TARGET_LINK_LIBRARIES(${TEST_NAME} VirtualRobot SimDynamics ${VirtualRobot_LINK_LIBRARIES} ${SimDynamics_PHYSICS_LIBRARIES})
    	IF(NOT UNIX)
    	   SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
    	ENDIF(NOT UNIX)
    	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "SimDynamics Tests")
    	ADD_TEST( SimDynamics_${TEST_NAME} ${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
    ENDMACRO(ADD_SIMDYNAMICS_TEST)
ENDIF(NOT SimDynamics_CONFIGURED)
