# STE with a Data-Driven Plume Model - **User Manual**

## Overview

The purpose of this document is to provide enough information for an easy integration and easy deployment of the DDPM project :)

There is a thorough explanation of this work in the papers published at ICRA 2023[1] and submitted to ICRA 2024[2]. Please make sure you read it before using the code. Some path in the code are hard-coded with absolute path that may cause the code to crash. Please first check if the path is correct and the destination folder exist before running. I recommend that, as the second thing to do, you try to run every step of the project without changing anything other than what is necessary (e.g., paths in the scripts). Don’t make significant changes before making sure everything runs smoothly.

There are many scripts, that I carefully commented (as far as I remember!). Please take the time to go through them before running them so you know what is happening. This way, if things don’t go as you expect, you know where the problem comes from. 

The software version I have used for this project are: OpenFoam v9, Cuda 12.0, python 3.8, gcc/g++ 9.4.0, libtorch-cxx11-abi-shared-with-deps-1.13.1+cu117

1. The pipeline of generating the training dataset and train the DDPM
    1. Environment maps: generate different environments with different obstacle configurations for training and testing dataset
    2. Wind maps: run OpenFoam simulation to get the wind profile for each environment
    3. Gas maps: export wind profile to Webots; gather the gas dispersion maps with static sensor nodes in each environment with different source positions
    4. Training/ Testing dataset: For each source position, combine environment definition, SDF and reverse distance to source as model input, and gas dispersion map as output
    5. Train the model
    6. Export model as .pt to be callable by libtorch
2. Combine the STE with DDPM – use DDPM as a surrogate model to get the prediction of the gas concentration at sampling point

Thus, if you don’t want to retrain the model but only use the DDPM and make the development for STE framework, you don’t need to touch the first part.

Since the project is composed of many parts, I will explain bellow how to run different parts and how the files are organized.

## How to run?

### DDPM – dataset generation, model training, model exporting

#### 1. OpenFOAM (wind maps)
a. **Training set (random maps)**

First, we need to create wind maps for the training set. The scripts to generate the random data training data set are in “training_random”. “training_random_generate(1/2/3).sh” makes new maps with random obstacles. “training_random_generate_more(1/2/3).sh” Make more obstacles, that are stuck to the walls, in a random way. Both these scripts are well commented for an easy understanding and modification. You can launch several script at the same time depends on the available CPU of your machine. For each script, it will run the parallel OpenFOAM with 4 CPU.


One thing to be sure about: after running these scripts, new folders appear with OpenFOAM data in them, you need to visualize these folders (by running matlab script openfoam_process_for_webots_random.m) to make sure that the obstacles don’t prevent the wind from going from inlet to outlet (the obstacle don't fullfill y axis), because otherwise the simulation won’t work.

If while going through the maps, you encounter such map, delete the folder, and re-run the script as it is. The script won’t overwrite the good maps, it will only create the one that is missing. Make sure that the new map is also valid. (If you can find a way to automatize this, it would be great!)

**b. Test set (manually defined maps)**

For the test set, the process is different, because we want to make the maps manually. The data and scripts are in the folder “test_maps”. First, you create a folder mapX by copying the map0 folder. Then the file “mapX/mapX_wind0/system/topoSetDict” needs to be modified to determine the obstacles in the test map. Then, to run all test maps, you can run the script “run_all_maps.sh”. This script does not overwrite, so if you want to re-run, you first need to remove the previous results with the script “remove_all_results.sh” or remove the bad results manually.

#### 2. MATLAB (process wind maps for Webots)
Once the wind maps are generated, we need to make them usable by webots to make the plume maps. The script “openfoam_process_for_webots_random.m” (located in the matlab folder) allows to process the randomly generated wind maps in OpenFOAM so we can easily load them on Webots. All you need to do is to run the script, but make sure the “general_folder” is correctly set. Also, the universal grid (lines 6-11) should be set according to what is set in OpenFOAM. The output of this script is a picture that shows 2 of the feature maps and the wind speed on x and y, as well as a txt file containing the wind information for each cell which is then read by Webots.
To do the same thing for the test set, you need to run the following script with the same detail as
above: “openfoam_process_for_webots_test_maps.m”.

#### 3. Webots (plume maps)
To generate plume maps with webots for the training data set, you need to run the script “training_data_generation_random_WB.sh” and for the test maps, the script “test_data_generation_random_WB.sh”. Please make sure that the lines 3 to 6 are set according to your settings. Line 9 allows to run the script for the maps number that you want to run, make sure it is set to “for d in {1..200}” if you want to run them all. For each wind map, we run the webots simulation 20 times (parameter j) with random source positions.
The results will be written in the “Results/training_random” folder, where for each map, 20 csv files are created.

#### 4. MATLAB (process plume maps to make input data for neural network)
Once the plume maps for the training data set are created, we need to pack them all as a data set for our neural network. The scripts that will do this respectively for the training set and the test set, are “prepare_dataset_random.m” and “prepare_dataset_test_maps.m”. They will create
.mat files (check out the end of the matlab script) that will be read by our Python script.

#### 5. Python (run the training and generate the model)
Finally, the learning part! Here we will use the training set to learn the model. The main Python script is “DeepCFD.py”. Make sure the lines 21 and 26 correspond to the .mat files created by matlab before. To run this script, you need to have a few things installed. I used Python 3.
The training takes about a week on my desktop computer for 1000 epochs. I used to set the epoch number to 300 for tests, but for final results, I set it to 1000. The output of this script if a .pt file which is the model (set on line 137 of DeepCFD.py). Now to get the output, we use the script “generate_output.py”, which loads the model as well as the .mat file for the test maps, and run the model on the test maps. The output is written in the “output” folder as pictures that show both the ground truth, the generated map and the error. Once you are satisfied with these results, you can take the model (the .pt file) and use it in your STE algorithm!Once the plume maps for the training data set are created, we need to pack them all as a data set for our neural network. The scripts that will do this respectively for the training set and the test set, are “prepare_dataset_random.m” and “prepare_dataset_test_maps.m”. They will create
.mat files (check out the end of the matlab script) that will be read by our Python script.
