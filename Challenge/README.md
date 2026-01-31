## MedGuide AI – Challenge Code Folder

This folder contains the **challenge-specific code** for the MedGuide project. It combines the already existing `lidar.launch` and `jetbot.launch` files with our **custom Python scripts** into a single main launch file named `medguide_ai.launch`. Consequently, all code presented in this folder was developed by us and represents our project-specific contribution.

### Prerequisites

To run this code, the following setup is required:

- A **Jupyter Notebook session** established on the JetBot (Jetson Nano acting as host)
- A **personal laptop** running the notebook  
  `live_demo_trt_medguide-ROS.ipynb`

This notebook is responsible for:
- Loading our **AI model**
- Predicting the **x and y control values**
- Visualizing these values directly in the camera feed

The predicted values are mapped to the robot’s motor commands and transmitted to the ROS system via **UDP**.

### Motivation for the Architecture

This architecture was necessary due to **Python version incompatibilities**:
- TensorFlow on the Jetson Nano is only supported **inside a Docker container**
- **ROS 1** is incompatible with the Python version required by TensorFlow

To resolve this conflict, inference and visualization are handled externally in the Jupyter environment, while ROS manages navigation and actuation.

### Launch Order

1. Start the Jupyter Notebook and ensure:
   - The AI model is loaded
   - UDP communication is active
   - Motor gains are correctly configured
2. Launch the ROS system on the Jetbot using:
   ```bash
   roslaunch medguide_ai.launch
   ```
3.  Only after these steps are completed will the full MedGuide AI pipeline operate correctly.
