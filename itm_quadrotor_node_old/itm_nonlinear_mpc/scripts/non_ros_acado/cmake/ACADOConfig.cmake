################################################################################
#
# Description:
#	ACADO Toolkit package configuration file
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2011 - 2013.
#
# NOTE:
#	- /
#
# Usage:
#	- /
#
################################################################################

################################################################################
#
# Configurable section
#
################################################################################

# 
# Tell the user project where to find our headers, libraries and external
# packages, etc.
#
SET( ACADO_INCLUDE_DIRS "C:/dev/github_ws/MPC_test/MPC_python/include/acado" )
SET( ACADO_LIBRARY_DIRS "C:/dev/github_ws/MPC_test/MPC_python/lib" )
SET( ACADO_EXTERNAL_PACKAGES_DIR "C:/dev/github_ws/MPC_test/MPC_python/external_packages" )
SET( ACADO_CMAKE_DIR "./cmake" )
 
#
# List of ACADO static libraries
#
SET( ACADO_STATIC_LIBRARIES acado_toolkit )
#
# List of ACADO shared libraries
#
SET( ACADO_SHARED_LIBRARIES  )

#
# ACADO is shipped with embedded version of qpOASES. Here is specified
# where source and header files reside
#
SET( ACADO_QPOASES_EMBEDDED_SOURCES C:/dev/github_ws/MPC_test/MPC_python/external_packages/qpoases/SRC/Bounds.cpp;C:/dev/github_ws/MPC_test/MPC_python/external_packages/qpoases/SRC/CyclingManager.cpp;C:/dev/github_ws/MPC_test/MPC_python/external_packages/qpoases/SRC/MessageHandling.cpp;C:/dev/github_ws/MPC_test/MPC_python/external_packages/qpoases/SRC/QProblem.cpp;C:/dev/github_ws/MPC_test/MPC_python/external_packages/qpoases/SRC/Utils.cpp;C:/dev/github_ws/MPC_test/MPC_python/external_packages/qpoases/SRC/Constraints.cpp;C:/dev/github_ws/MPC_test/MPC_python/external_packages/qpoases/SRC/Indexlist.cpp;C:/dev/github_ws/MPC_test/MPC_python/external_packages/qpoases/SRC/QProblemB.cpp;C:/dev/github_ws/MPC_test/MPC_python/external_packages/qpoases/SRC/SubjectTo.cpp;C:/dev/github_ws/MPC_test/MPC_python/external_packages/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp )
SET( ACADO_QPOASES_EMBEDDED_INC_DIRS C:/dev/github_ws/MPC_test/MPC_python/external_packages/qpoases/;C:/dev/github_ws/MPC_test/MPC_python/external_packages/qpoases/INCLUDE;C:/dev/github_ws/MPC_test/MPC_python/external_packages/qpoases/SRC )

################################################################################
#
# Validation
#
################################################################################

# TODO

