#**Digital Twin Robot: armpi_fpv**

##*Training AI models for specific motor error détection based on datasets generated via digital twin simulation.*

#**Introduction**

The aim of this project was to detect specific motor errors in a robot arm type robot, pury by observation of performed trajectories and access to the original command trajectory. In order to this our goal was to train an AI with satisfactory detection performance aka being able to detect which motor the motor error originated from the type of motor error that took place.

We collaborated with MathLab during this project, their intent being to use our project as a tutorial for the creation inside a full matlab pipeline of a digital twins in simulink, it's command and the training of adapted AI models. 

Given this collboration most of our work was performed inside of MatLab. 

##**How does the simulations work?**

Currently (although this can be modified as described further on) each trajectory simulated lasts 10 seconds with a 100 points to attained per second, leading to each 3 dimensional trajectory being described as a 3*1000 matrix in a [[x1,..,x1000],[y1,..,y1000],[z1,..,z1000]] format.
The motor errors we have decided to study (described in detail in the comments of Main_final.m function) are total stop, stutter, lag, speed-cap, and steady state errors.
 The main idea behind our decision to choose this short timespan was to train our AI as to enable rapid sub 10 second detection of motor errors through their effect on trajectory. Due to this short timespan we have pre-supposed the only 1 motor may be affected by a motor error at a time as it seems improbable that two motors experience the type of motor errors we have focused on at a less than 10 second interval.

##**How to use the project?**

*Schematic overview of the main components of our project:*

- .slx files (model and sub-model) generated from a URDF file that represents the digital twin of the robot, as well as all the systems put in place to emulate the motor control on the real life robot.

- Main_final.m  file, main user interface, where a function enables through it's many option arguments to interact with almost all of the code of the project: creation of a simulated trajectory dataset, trajectory type and number of trajectories per type for the datasetk, various parameters regarding the simulation itself, choice of the type AI model to be trained (bi-lateral lstm, siamese network, gated-transformer network). Details on the options of the main function are in the descriptive comments.

- AI model .m files, there the files called by the previous Main_final.m file, any modification to hyperparameters or model architectures should be done in the file corresponding to modified model, additional model files can be created taking inspiration on the previous ones for interaction with the main code.


##**Extra Features:**

There are many extra files in the project that have served for printing results, or further research.
For example the optimal_train file, that can be treaded as a AI model architecture file, by varying training and validation length (in terms of longer of points inside the timeseries, as these are "cut up" from their original lenght of 1000 points) it aims to find the optimal paramters that ensure best perfomance on 3 rather large and varied pre-generated datasets.


