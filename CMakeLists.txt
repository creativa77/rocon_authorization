##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_authorization)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
  DIRECTORY msg
  FILES
  User.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetUsersRoles.srv
  SetUsersRoles.srv
)

file(GLOB_RECURSE ${PROJECT_NAME}_MODULE_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
file(GLOB_RECURSE ${PROJECT_NAME}_SCRIPT_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} scripts/*)

catkin_python_setup()

generate_messages(DEPENDENCIES)

catkin_package(CATKIN_DEPENDS message_runtime)

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS
        scripts/users_manager.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
