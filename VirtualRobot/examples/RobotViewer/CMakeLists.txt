PROJECT ( RobotViewer )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)
CMAKE_POLICY(VERSION 2.6)

############################# Setup paths to Simox #############################
GET_FILENAME_COMPONENT (CurrentPath ${CMAKE_CURRENT_LIST_FILE} PATH)

# Either you set your environment variable or you edit the path here manually
SET(SIMOX_DIR_STANDARD "${CurrentPath}/../../..")
#SET(SIMOX_DIR_STANDARD "$ENV{Simox_DIR}")

# be sure to have the absolute path
get_filename_component(SIMOX_DIR_STANDARD ${SIMOX_DIR_STANDARD} ABSOLUTE)

SET (SIMOX_DIR ${SIMOX_DIR_STANDARD} CACHE STRING "Path to Simox used by ${PROJECT_NAME}") 

# where is the build directory located
SET (SIMOX_BUILD_DIRECTORY ${SIMOX_DIR}/build CACHE STRING "Path to Simox build directory")

# include simox settings (handles lib search)
INCLUDE(${SIMOX_DIR}/config.cmake)
INCLUDE(${SIMOX_DIR}/CMakeModules/SimoxProject.cmake)

# Makes the demo directory available in your code 
SET(DEMO_BASE_DIR ${PROJECT_SOURCE_DIR})
ADD_DEFINITIONS(-DDEMO_BASE_DIR="${DEMO_BASE_DIR}") 

############################# The executable needs Coin3d/Qt #############################
IF(SIMOX_USE_COIN_VISUALIZATION)

   ############################# SET YOUR FILES HERE #############################
   # Adjust these lines according to your filenames 
   FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/RobotViewer.cpp ${PROJECT_SOURCE_DIR}/showRobotWindow.cpp)
   FILE(GLOB demo_INCS ${PROJECT_SOURCE_DIR}/showRobotWindow.h)

   ############################# OPTIONAL: Qt-related stuff #############################
   # if you have a Qt-based project, you have to moc the QMainWindow class file
   set(GUI_MOC_HDRS ${PROJECT_SOURCE_DIR}/showRobotWindow.h)

   # When you are using QtDesigner to build the UI, you can specify all .ui files here
   set(GUI_UIS ${PROJECT_SOURCE_DIR}/RobotViewer.ui)
   
   ############################# Build an executable #############################
   # pass file lists to simox cmake script: create the executable
   simox_add_executable(${PROJECT_NAME} "${demo_SRCS}" "${demo_INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
   SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")
   
ENDIF(SIMOX_USE_COIN_VISUALIZATION)
