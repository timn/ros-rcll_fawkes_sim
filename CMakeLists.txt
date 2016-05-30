cmake_minimum_required(VERSION 2.8.3)
project(rcll_fawkes_sim)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rcll_fawkes_sim_msgs
	fawkes_msgs
  roscpp
)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(INTERFACES ZoneInterface Position3DInterface TagVisionInterface RobotinoLightInterface NavGraphWithMPSGeneratorInterface)
find_package(Fawkes COMPONENTS CPP11 core interface blackboard ${INTERFACES})
if (NOT Fawkes_FOUND)
	message(FATAL "Fawkes could not be found")
else()
	set(NAVIFFILE "${Fawkes_BASEDIR}/src/plugins/navgraph-generator-mps/interfaces/NavGraphWithMPSGeneratorInterface.xml")
	if (EXISTS ${NAVIFFILE})
		FILE(READ ${NAVIFFILE} TMPTXT)
		STRING(FIND "${TMPTXT}" "name=\"id\"" matchres)
		if (NOT ${matchres} EQUAL -1)
			set(CFLAGS_EXTRA "-DHAVE_OLD_NAVGRAPH_GENMPS_INTERFACE")
		endif()
	endif()
endif()


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rcll_fawkes_sim
#  CATKIN_DEPENDS rcll_fawkes_sim_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(rcll_fawkes_sim src/rcll_fawkes_sim_node.cpp)

# Use Fawkes' protobuf_comm
set_target_properties(rcll_fawkes_sim PROPERTIES
  INCLUDE_DIRECTORIES "${Fawkes_INCLUDE_DIRS};${catkin_INCLUDE_DIRS}"
  COMPILE_FLAGS "${Fawkes_CFLAGS} ${catkin_CFLAGS} ${CFLAGS_EXTRA}"
  LINK_FLAGS "${Fawkes_LFLAGS}"
  LINK_LIBRARIES "${Fawkes_LIBRARIES};${catkin_LIBRARIES}"
  INSTALL_RPATH "${Fawkes_LIBRARY_DIRS}"
  INSTALL_RPATH_USE_LINK_PATH TRUE
  BUILD_WITH_INSTALL_RPATH TRUE)
