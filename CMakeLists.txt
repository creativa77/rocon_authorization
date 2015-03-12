##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_authorization)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS)
catkin_package()

#file(GLOB_RECURSE ${PROJECT_NAME}_MODULE_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
#file(GLOB_RECURSE ${PROJECT_NAME}_SCRIPT_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} scripts/*)

##############################################################################
# Installs
##############################################################################

install(DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
