# General Repository information

This Github repository contains the codebase for the paper (LINK HERE).

## Repository structure and contents 

it can be seperated into in two sections : <br />
  -one dedicated to interacting with the Armpi_FPV robot, and using it to generate .csv folders containing commands and responses on the real robot. This one contains the "shadow fork" which itself contains basic instructions on setting up data capture. (@Zhiguo : maybe move the whole stuff from the shadow fork here, to have a single unified git rather than two seperate ones - then just add my final "concatenated dataset builder" on robot side, I redid it so that it should be independant, w/ comments to explain what it does and how to change the parameters) <br />
  -one dedicated to using a digital twin of said robot in Matlab for LSTM training, and for applying necessary post-processing on real data to make predictions. Please refer to the "ik_model" folder for the specific README presenting the various files. <br />

This repository also contains sample datasets for both real trajectories captured on the robot, provided as per the capture code (with seperated command, duration and response position .csv files); and simulated data, provided as 3D coordinates for both command and response trajectories. These are correctly formatted to interact with our codes in the "ik_model" section, and may be used to either replicate our results, or as a reference to develop alterantive training and evaluation codes.

## Further information

Please address any questions regarding the repository to: <br />
killian.mccourt@free.fr <br />

@Zhiguo : I don't know whether any legal info (funding, etc... needs to feature here)



