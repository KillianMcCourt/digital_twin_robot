This file references the steps we are currently working on.

(- study command & response conversion to a 1000 points

- use single generation & postprocessing data split)

1- training AI model on the 4/6 direct motor outputs (j1,...,j6) instead of the x,y,z coordinates of the end effector of the robot, expected great increase in AI model performance


2- given the initial objective to base the predictions upon the observation of the  x,y,z coordinates of the end effector of the robot, compare previous in 1- with performance obtained from non real motor outputs, reconstructed a posteriori from  x,y,z coordinates via a inverse cinématics operation (this observation based on guesses is not a bijection due to the supplémentaries degrees of freedom of the system and therefore a decrease in performance is exepected)


3- verify is python and matlab trajectory generation coincide one with another, in the case that they don't set up a pipeline to replace inital pre-simulation trajectory generation in matlab by same process in python to align AI model testing and training trajectories to  in use trajectories. If these trajectories do coincide it would still be "cleaner" to set up this pipeline but it will not be a priority at all.


