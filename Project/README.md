## Project Description

**TBD**

For mapping, the `slam.launch` file was executed on the **JetBot (master)**, while a **virtual machine (slave)** was used to run **RViz** in order to visualize both the **camera feed** and the **LiDAR data**. After mapping the entire environment, the resulting map was saved in the `./maps` directory as `project.yaml`.

This map is subsequently used in `voice_nav.launch`, which integrates all required ROS nodes, including **AMCL**, **move_base**, **LiDAR**, **CSI camera**, and **JetBot motor control**. The launch file relies on the `voice_nav.py` script to perform **voice recognition** as well as **text-to-speech (TTS)**, enabling voice-based navigation within the mapped environment.
