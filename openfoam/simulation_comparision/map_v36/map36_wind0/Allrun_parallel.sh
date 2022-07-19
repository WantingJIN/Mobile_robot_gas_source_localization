#!/bin/sh

cd ${0%/*} || exit 1    # Run from this directory

# Source tutorial run functions
. $WM_PROJECT_DIR/bin/tools/RunFunctions

#to generate surface feature from system/surfaceFeaturesDict
runApplication surfaceFeatures

#to create the mesh from system/blockMeshDict
runApplication blockMesh

runApplication decomposePar -copyZero

runParallel snappyHexMesh -overwrite

#to log cell centers for post processing
#writeCellCentres
#runApplication postProcess -func writeCellCentres

# runParallel renumberMesh -overwrite
runParallel patchSummary
runParallel potentialFoam
#to run the solver and generate results
# Get application name
runParallel $(getApplication)

runApplication reconstructParMesh -constant
runApplication reconstructPar
#to generate VTK files for paraview
foamToVTK
