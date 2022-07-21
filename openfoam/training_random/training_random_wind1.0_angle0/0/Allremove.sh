#!/bin/sh

cd ${0%/*} || exit 1    # Run from this directory

# Source tutorial clean functions
. $WM_PROJECT_DIR/bin/tools/CleanFunctions

cleanCase
cleanExplicitFeatures
rm map*_wind*.txt
rm map*_wind*.png
rm -r VTK
rm 0/C*
rm constant/triSurface/*.eMesh
