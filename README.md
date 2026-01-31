# bip-ai-in-robotics

This repository contains all code developed by **Team 11** for the **Blended Intensive Program (BIP) – Artificial Intelligence in Robotics** project.

## Repository Structure

- **`challenge/`**  
  Contains the implementation of the **line-following behavior**, developed as part of the challenge task. This folder focuses on visual perception and control for following a marked path.

- **`project/`**  
  Showcases the **MedGuide system**, including **voice recognition**, **SLAM-based navigation**, **AMCL** and autonomous guidance functionalities within a hospital-like environment.

- **`training/`**  
  Includes all scripts and notebooks used to **train the neural network model** used in the line-following module. This covers data preparation, training, and evaluation.

## Notes on Folder Structure

The full ROS `src/` directory is **intentionally not included** in this repository, as it is **preinstalled on the JetBot platform**. Users are expected to **set up the correct folder structure manually** on their own system and integrate the provided code into the existing JetBot ROS workspace.

## Future Integration

As part of the ongoing MedGuide development, the **line-following approach will be combined with the voice-based navigation system**, enabling seamless interaction between perception, speech commands, and autonomous navigation.
