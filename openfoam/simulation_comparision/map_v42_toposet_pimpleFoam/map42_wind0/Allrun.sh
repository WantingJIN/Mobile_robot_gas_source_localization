#!/bin/sh

#to create the mesh from system/blockMeshDict
blockMesh

#to add the obstacles defined in system/topoSetDict
topoSet

#to add walls around obstacles
subsetMesh -overwrite c0 -patch walls

#to log cell centers for post processing
#writeCellCentres
postProcess -func writeCellCentres

#to run the solver and generate results
simpleFoam > log

#to generate VTK files for paraview
foamToVTK
