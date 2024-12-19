# Copyright (C) 2013-2018 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

cmake_minimum_required(VERSION 3.16.0)

project(MomentumBasedTorqueControl CXX)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

# Enable RPATH support for installed binaries and libraries
option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)

# Encourage user to specify a build type (e.g. Release, Debug, etc.), otherwise set it to Release.
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()


################################################################################
########################## Mandatory dependencies ##############################

find_package(iDynTree 12.3.1 REQUIRED)

find_package(BipedalLocomotionFramework 0.19.0 REQUIRED)

find_package(Eigen3 3.2.92 REQUIRED)

find_package(OsqpEigen REQUIRED)

find_package(QpSolversEigen REQUIRED)

########################## Optional dependencies ##############################

find_package(YARP QUIET)

find_package(matioCpp QUIET)

find_package(BipedalLocomotionFramework COMPONENTS VectorsCollection REQUIRED)

set(CONTROLLER_HEADERS include/MomentumBasedTorqueControl/Controller.h include/MomentumBasedTorqueControl/QPproblem.h include/MomentumBasedTorqueControl/ReferenceGenerator.h)

set(CONTROLLER_SOURCES src/Controller.cpp src/QPproblem.cpp src/ReferenceGenerator.cpp src/main.cpp)
                 
add_executable(${PROJECT_NAME} ${CONTROLLER_SOURCES} ${CONTROLLER_HEADERS})

target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
    
target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_os
    BipedalLocomotion::ParametersHandlerYarpImplementation
    BipedalLocomotion::RobotInterfaceYarpImplementation
    BipedalLocomotion::TextLogging
    BipedalLocomotion::FloatingBaseEstimators
    BipedalLocomotion::SystemYarpImplementation
    BipedalLocomotion::ContinuousDynamicalSystem
    BipedalLocomotion::YarpUtilities
    BipedalLocomotion::System
    iDynTree::idyntree-high-level
    BipedalLocomotion::VectorsCollection
    QpSolversEigen::QpSolversEigen)
    
add_subdirectory(config)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

include(AddUninstallTarget)





