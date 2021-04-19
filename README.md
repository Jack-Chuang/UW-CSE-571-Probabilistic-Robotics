# Overview of cse571-sp21-project-1 repository

### cse571-sp21-project-1 repo contains four packages that you won't have to modify:
- `my_at_localization` that localizes the AprilTAGs seen by the robot
- `my_static_tf` that publishes coordinate transforms (position and orientation) of all the landmarks in predetermined maps. These coordinate frames are in the form of ROS `\tf` messages
- `my_viz` that can publish map information and path traced by the robot in a visualizer (ROS visualizer called RViz).
- `image_processing` that contains utility functions to process images.

### cse571-sp21-project-1 repo contains TODO packages that you will have to edit in order to complete the project:
- `my_kf_localization` that localizes the robot based on the velocity control and AprilTAG localization.

You can see them in the `./packages/` path.
The source code for each of these packages are inside their respective `src/` folders. For example for `my_at_localization` you will see the source code in the path `./packages/my_at_localization/src/`.

### [DO NOT HAVE TO DO] To understand the folder structure and how to create new packages, build and run the packages on local as well as on robot, please see the instructions below.
[Package building documentation] (https://docs.duckietown.org/daffy/duckietown-robotics-development/out/dt_infrastructure.html)

# Build the provided packages and run it on the robot
After completing project 0.5. You must be able to communicate with your robot from your local computer using `dts shell`

### Clone the cse571-sp21-project-1 repository on to your local computer
`git clone https://github.com/fishbotics/cse571-sp21-project-1.git
cd cse571-sp21-project-1`

### Build the above mentioned packages on the duckiebot
`dts dts devel build -f -H {ROBOT_NAME}.local` from the `{PATH}\cse571-sp21-project-1` folder.

### Run the packages on the duckiebot.
`dts devel run -H {ROBOT_NAME}.local` from the `{PATH}\cse571-sp21-project-1` folder.

### Visualize what is happening by logging into the robot with the below command.
`dts start_gui_tools --vnc ROBOT_NAME` and open `http://localhost:8087/`
Open the Rviz, add `\tf` from the panel.
[Here is the video of what it will look like](Video)
[TODO] Include an image/video of the Rviz.

### How did the run command know what to run? Change what you are launching with the above run command.
By default, the above command will launch the package(s) indicated in the `./launchers/default.sh` file. 

NOTE: Be sure to follow these [instructions](https://docs.duckietown.org/daffy/duckietown-robotics-development/out/dt_infrastructure.html) in order to build. You cannot build this on your desktop using the typical `docker build` workflow. 

# Additional Documentation from Template:
### 1. Define dependencies

List the dependencies in the files `dependencies-apt.txt` and
`dependencies-py3.txt` (apt packages and pip packages respectively).

### 2. Place your code

Place your code in the directory `/packages/` of
your new repository.

### 3. Setup launchers

The directory `/launchers` can contain as many launchers (launching scripts)
as you want. A default launcher called `default.sh` must always be present.

If you create an executable script (i.e., a file with a valid shebang statement)
a launcher will be created for it. For example, the script file 
`/launchers/my-launcher.sh` will be available inside the Docker image as the binary
`dt-launcher-my-launcher`.

When launching a new container, you can simply provide `dt-launcher-my-launcher` as
command.
