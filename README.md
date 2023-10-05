# Rapidly Exploring Random Tree generator

## Table of Contents
- [Introduction](#introduction)
- [Project Files](#project-files)
- [Additional Resources](#additional-resources)

## Introduction
This repository contains a code source that generates a Rapidly Exploring Random Tree (RRT) and visualises it using Pygame.

The implemented algorithm generates what is called an RRT: a multi-query path planners that takes as input the start and generate a tree structure that keeps on expanding into the free space (without taking the robot's geometry into consideration), with a bias towards the goal, until it is reached.

The visualisation of this algorithm is done using the Pygame cross-platform library. The obstacles are considered of a square shape with costumable number and dimensions (set to a medium level by default).

## Project Files
The project includes the following components:

- **Code Source:** Two (2) Python files :

[RRT_base](./RRT_base.py) that contains two modules containing functions for generation and visualisation of the tree. Please make sure both are executable before running them.

[RRT_main](./RRT_main.py) that integrates all the functions prepared in the other file to properly implement the RRT algorithm.

- **Simulation Video:** A video demonstrating the simulation of the generated RRT (with default settings on the obstacles and the start-goal pair). You can watch the video [here](./demo_video.mp4).

## Additional Resources
For additional context and information about the project, you can watch the original tutorial the was followed into implementing this code [here](https://youtube.com/playlist?list=PL9RPomGb9IpRlfQEGkWnTt8jIauPovpOH&si=I5NM_JdnyWiOX2H5)

Also, feel free to reach out :
- [Linkedin Profile](https://www.linkedin.com/in/yhadj/)
- [Email](mailto:yasser.hadj@g.enp.edu.dz)