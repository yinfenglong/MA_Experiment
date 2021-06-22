################################################################################
#
# Description:
#	ACADO Toolkit package configuration file
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2011 - 2014.
#
# NOTE:
#	- Linux/Unix only.
#
# Usage:
#	- Linux/OSX:
#		* Users are supposed to source this file into ~/.bashrc (or ~/.profile).
#
################################################################################

################################################################################
#
# Definitions for both users and developers.
#
################################################################################

# 
# Tell the user project where to find our headers, libraries and external
# packages, etc.
#
export ACADO_ENV_INCLUDE_DIRS="/media/psf/Home/Develop/ACADOtoolkit;/media/psf/Home/Develop/ACADOtoolkit/acado;/media/psf/Home/Develop/ACADOtoolkit/external_packages;/media/psf/Home/Develop/ACADOtoolkit/external_packages/qpOASES-3.2.0/include;/media/psf/Home/Develop/ACADOtoolkit/build_linux"
export ACADO_ENV_LIBRARY_DIRS="/media/psf/Home/Develop/ACADOtoolkit/build_linux/lib"
export ACADO_ENV_EXTERNAL_PACKAGES_DIR="/media/psf/Home/Develop/ACADOtoolkit/external_packages"
export ACADO_ENV_CMAKE_DIR="/media/psf/Home/Develop/ACADOtoolkit/cmake"
 
#
# List of ACADO static libraries
#
export ACADO_ENV_STATIC_LIBRARIES=""
#
# List of ACADO shared libraries
#
export ACADO_ENV_SHARED_LIBRARIES="acado_toolkit_s"

#
# ACADO is shipped with embedded version of qpOASES. Here is specified
# where source and header files reside
#
export ACADO_ENV_QPOASES_EMBEDDED_SOURCES="/media/psf/Home/Develop/ACADOtoolkit/external_packages/qpoases/SRC/Bounds.cpp;/media/psf/Home/Develop/ACADOtoolkit/external_packages/qpoases/SRC/CyclingManager.cpp;/media/psf/Home/Develop/ACADOtoolkit/external_packages/qpoases/SRC/MessageHandling.cpp;/media/psf/Home/Develop/ACADOtoolkit/external_packages/qpoases/SRC/QProblem.cpp;/media/psf/Home/Develop/ACADOtoolkit/external_packages/qpoases/SRC/Utils.cpp;/media/psf/Home/Develop/ACADOtoolkit/external_packages/qpoases/SRC/Constraints.cpp;/media/psf/Home/Develop/ACADOtoolkit/external_packages/qpoases/SRC/Indexlist.cpp;/media/psf/Home/Develop/ACADOtoolkit/external_packages/qpoases/SRC/QProblemB.cpp;/media/psf/Home/Develop/ACADOtoolkit/external_packages/qpoases/SRC/SubjectTo.cpp;/media/psf/Home/Develop/ACADOtoolkit/external_packages/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp"
export ACADO_ENV_QPOASES_EMBEDDED_INC_DIRS="/media/psf/Home/Develop/ACADOtoolkit/external_packages/qpoases/;/media/psf/Home/Develop/ACADOtoolkit/external_packages/qpoases/INCLUDE;/media/psf/Home/Develop/ACADOtoolkit/external_packages/qpoases/SRC"

