# ardupilot-test


### PART 1 - ARDUPILOT INSTALLATION (WITH SITL-Software In The Loop) AND ARDUCOPTER TEST ON UBUNTU 22.04 LTS
> **NOTE:** You can also check out the links to understand this step
> 
> [ardupilot setup doc](https://ardupilot.org/dev/docs/building-setup-linux.html)
> 
> [ardupilot SITL youtube tutorial](https://www.youtube.com/watch?v=c1z6mSps2nI&list=PLucyk5x5RZwvFN5UylEavMfLrvtREwFEW)

- Ensure you have git installed on your PC
  > ``` shell
  > sudo apt install git
  > ```

- Create a folder for your ardupilopt
  > ``` shell
  > mkdir ~/ardupilot-test
  > ```

- Clone the main ardupilot branch in the ardupilot-test folder (this might take a while depending on your Internet speed)
  > ``` shell
  > cd ~/ardupilot-test
  > git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
  > ```

- Install required packages (while in the downloaded/cloned ardupilot folder)
  > ``` shell
  > cd ~/ardupilot-test/ardupilot
  > Tools/environment_install/install-prereqs-ubuntu.sh -y
  > . ~/.profile #add the path to profile
  > ```

- [once done **log-out** and **log-in** again to make it permanent]

- Build/configure ArduPilot for SITL (Software In The Loop) board
  > ``` shell
  > cd ~/ardupilot-test/ardupilot
  > ./waf configure --board sitl
  > ```

- Build/configure the arduipilot vehicle type to be used - ArduCopter (copter) (NOTE: this might take a while depending on your machine resources)
  > ``` shell
  > cd ~/ardupilot-test/ardupilot
  > ./waf copter
  > ```

- Test run with the copter simulation with SITL (you can CTRL-C to close)
  > ``` shell
  > cd ~/ardupilot-test/ardupilot/Tools/autotest
  > python3 sim_vehicle.py -v copter --console --map
  > ```


#

### PART 2 - ROS2 (ros-humble) INSTALLATION

> **NOTE:**
> If you have not installed **ros-humble** on your **ubuntu 22.04**, pls follow this [tutorial](https://robocre8.gitbook.io/robocre8/tutorials/how-to-install-ros2-humble-desktop-on-pc-full-install) to do so
> By the end of the tutorial, you will have installed ros-humble and the necessary tools to build your packages.
>
> If you didn't follow the above tutorial, you'll need to install **rosdep** and **vcstool**
> ```shell
> sudo apt install ros-dev-tools
> ```

#

### PART 3 - ARDUPILOT AND ARDUCOPTER ROS2 (ros-humble) SETUP AND TEST

- Create an ArduPilot ROS2 workspace
  > ``` shell
  > mkdir -p ~/ardu_ws/src
  > ```

- Clone the required repository using vcs (please, this might take a while, be patient)
  > ``` shell
  > cd ~/ardu_ws/
  > vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
  > ```

- Install the necessary ArduPilot ROS dependencies
  > ``` shell
  > cd ~/ardu_ws
  > sudo apt update
  > rosdep update
  > source /opt/ros/humble/setup.bash
  > rosdep install --from-paths src --ignore-src -r -y
  > ```

- Install the MicroXRCEDDSGen build dependency:
  > ``` shell
  > sudo apt install default-jre
  > cd ~/ardu_ws
  > git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
  > cd Micro-XRCE-DDS-Gen
  > ./gradlew assemble
  > export PATH=\$PATH:$PWD/scripts
  > echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc
  > ```

- You can test the MicroXRCEDDSGen installation
  > ``` shell
  > source ~/.bashrc
  > microxrceddsgen -help
  > ```

- Now build the ardu_ws ROS workspace (preferably open a new terminal):
  > ``` shell
  > cd ~/ardu_ws
  > colcon build --packages-up-to ardupilot_dds_tests
  > ```

- If the build fails, when you request help, please re-run the build in verbose mode like so:
  > ``` shell
  > cd ~/ardu_ws
  > colcon build --packages-up-to ardupilot_dds_tests --event-handlers=console_cohesion+
  > ```

- If you’d like to test your ArduPilot ROS 2 installation, run:
  > ``` shell
  > cd ~/ardu_ws
  > source ./install/setup.bash
  > colcon test --executor sequential --parallel-workers 0 --base-paths src/ardupilot --event-handlers=console_cohesion+
  > colcon test-result --all --verbose
  > ```

- Now build the ardupilot SITL package(s)
  > ``` shell
  > cd ~/ardu_ws
  > colcon build --packages-up-to ardupilot_sitl
  > ```

- Let's test run the ArduPilot SITL
  > ``` shell
  > source ./install/setup.bash
  > ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501
  > ```
  
#

### PART 4 - ARDUPILOT AND ARDUCOPTER ROS2 GAZEBO (ignition-hermonic) SIMULATION SETUP AND TEST

- Install Gazebo Harmonic (with ros-humble)
  > ``` shell
  > sudo apt-get update
  > sudo apt-get install curl lsb-release gnupg
  > sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  > echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  > sudo apt-get update
  > sudo apt-get install gz-harmonic
  > ```

- Set the Gazebo version to harmonic (recommended). It’s recommended to set this in your ~/.bashrc file.
  > ``` shell
  > export GZ_VERSION=harmonic
  > echo "export GZ_VERSION=harmonic" >> ~/.bashrc
  > ```

- Add Gazebo sources to rosdep for the non-default pairing of ROS 2 Humble and Gazebo Harmonic.
  > ``` shell
  > sudo wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list
  > rosdep update
  > ```

- Clone the ArduPilot simulation repo
  > ``` shell
  > cd ~/ardu_ws
  > vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
  > rosdep update
  > rosdep install --from-paths src --ignore-src -y
  > ```

- Build the ArduPilot ROS2 simulation package
  > ``` shell
  > cd ~/ardu_ws
  > colcon build --packages-up-to ardupilot_gz_bringup
  > ```

- If you’d like to test your installation, run:
  > ``` shell
  > cd ~/ardu_ws
  > source install/setup.bash
  > colcon test --packages-select ardupilot_sitl ardupilot_dds_tests ardupilot_gazebo ardupilot_gz_applications ardupilot_gz_description ardupilot_gz_gazebo ardupilot_gz_bringup
  > colcon test-result --all --verbose
  > ```

- Run the ardupilot simulation (read more here: https://github.com/ArduPilot/ardupilot_gz#ardupilot_gz)
  > ``` shell
  > cd ~/ardu_ws
  > export GZ_VERSION=harmonic
  > source install/setup.bash
  > ros2 launch ardupilot_gz_bringup iris_runway.launch.py
  > ```
