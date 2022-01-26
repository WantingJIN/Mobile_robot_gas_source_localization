# How to run

## openfoam

1. **Generate**: `bash training_random_generate.sh`

    create training data with different kind of obstacles

2. (optional) `bash training_random_generate_more.sh`

   create more training data with obstacles stuck to the wall

3. **Visualization**: matlab `openfoam_process_for_webots_random.m`

Note: To visualize the openfoam simulation result in ParaView, the embedded command with paraFoam didn't work well for me. Thus I directly lunch paraview and open vtk file from there.

## Webots

​    

​    

