# ardupilot-test


### PART1 - ARDUPILOT INSTALLATION (WITH SITL-Software In The Loop) AND ARDUCOPTER TEST ON UBUNTU 22.04 LTS
> **NOTE:** you can also checkout the links to understand this step
> 
> [ardupilot setup doc](https://ardupilot.org/dev/docs/building-setup-linux.html)
> 
> [ardupilot SITL youtube tutorial](https://www.youtube.com/watch?v=c1z6mSps2nI&list=PLucyk5x5RZwvFN5UylEavMfLrvtREwFEW)

- ensure you have git installed on your PC
  > ``` shell
  >  sudo apt install git
  > ```

- create a folder for your ardupilopt
  > ``` shell
  >  mkdir ~/ardupilot-test
  > ```

- change directory into the ardupilot-test folder
  > ``` shell
  >  cd ~/ardupilot-test
  > ```

- clone the main ardupilot branch in the ardupilot-test folder (this might take a while depending on your Internet speed)
  > ``` shell
  >  git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
  > ```

- change directory into the downloaded/cloned ardupilot folder
  > ``` shell
  >  cd ~/ardupilot-test/ardupilot
  > ```

- install required packages (while in the downloaded/cloned ardupilot folder)
  > ``` shell
  >  cd ~/ardupilot-test/ardupilot
  >  Tools/environment_install/install-prereqs-ubuntu.sh -y
  >  . ~/.profile #add the path to profile
  > ```

- [once done **log-out** and **log-in** again to make it permanent]

- build/configure arduipilot for SITL (Software In The Loop) board
  > ``` shell
  >  cd ~/ardupilot-test/ardupilot
  >  ./waf configure --board sitl
  > ```

- build/configure arduipilot vehicle type to be used - ArduCopter (copter) (NOTE: this might take a while depending on your machine resources)
  > ``` shell
  >  cd ~/ardupilot-test
  >  cd ~/ardupilot-test/ardupilot
  >  ./waf copter
  > ```

- Test run with the copter simulation with SITL (you can CTRL-C to close)
  > ``` shell
  >  cd ~/ardupilot-test
  >  cd ~/ardupilot-test/ardupilot/Tools/autotest
  >  python3 sim_vehicle.py
  > ```


#

### PART2 - ROS2 (ros-humble) INSTALLATION

> **NOTE:**
> If you have not installed **ros-humble** on your **ubuntu 22.04**, pls follow this [tutorial](https://robocre8.gitbook.io/robocre8/tutorials/how-to-install-ros2-humble-desktop-on-pc-full-install) to do so
> By the end of the tutorial, you will have installed ros-humble and the necessary tools to build your packages.
>
> If you didn't follow the above tutorial, you'll need to install **rosdep** and **vcstool**
> ```shell
>  sudo apt install ros-dev-tools
> ```

#

### PART3 - ARDUPILOT AND ARDUCOPTER ROS2 (ros-humble) SETUP AND TEST

create ardupilot ros2 workspace
-> mkdir -p ~/ardu_ws/src

clone the required reposotory using vcs (please this might take a while be patient)
-> cd ~/ardu_ws/
-> vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src

install the necessary ardupilot ros dependencies
-> cd ~/ardu_ws
-> sudo apt update
-> rosdep update
-> source /opt/ros/humble/setup.bash
-> rosdep install --from-paths src --ignore-src -r -y


now build the ardu_ws ros workspace:
-> cd ~/ardu_ws
-> colcon build --packages-up-to ardupilot_dds_tests

If the build fails, when you request help, please re-run the build in verbose mode like so:
-> colcon build --packages-up-to ardupilot_dds_tests --event-handlers=console_cohesion+

If you’d like to test your ArduPilot ROS 2 installation, run:
-> cd ~/ardu_ws
-> source ./install/setup.bash
-> colcon test --executor sequential --parallel-workers 0 --base-paths src/ardupilot --event-handlers=console_cohesion+
-> colcon test-result --all --verbose

now build the ardupilot SITL package(s)
-> cd ~/ardu_ws
-> colcon build --packages-up-to ardupilot_sitl

let's test run the arduipilot sitl 
-> source ./install/setup.bash
-> ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501


PART4 - ARDUPILOT AND ARDUCOPTER ROS2 GAZEBO (igintion-hermonic) SIMULATION SETUP AND TEST

install gazebo harmonic (with ros-humble)
-> sudo apt-get update
-> sudo apt-get install curl lsb-release gnupg
-> sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
-> echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
-> sudo apt-get update
-> sudo apt-get install gz-harmonic

Set the Gazebo version to harmonic (recommended) It’s recommended to set this in your ~/.bashrc file.
-> export GZ_VERSION=harmonic
-> echo "export GZ_VERSION=harmonic" >> ~/.bashrc

Add Gazebo sources to rosdep for the non-default pairing of ROS 2 Humble and Gazebo Harmonic.
-> sudo wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list
-> rosdep update

clone the arduipilot simulation repo
-> cd ~/ardu_ws
-> vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
-> rosdep update
-> rosdep install --from-paths src --ignore-src -y

Build the ardupilot ros2 simulation package
-> cd ~/ardu_ws
-> colcon build --packages-up-to ardupilot_gz_bringup

If you’d like to test your installation, run:
-> cd ~/ardu_ws
-> source install/setup.bash
-> colcon test --packages-select ardupilot_sitl ardupilot_dds_tests ardupilot_gazebo ardupilot_gz_applications ardupilot_gz_description ardupilot_gz_gazebo ardupilot_gz_bringup
-> colcon test-result --all --verbose

run the ardupilot simulation (read more here: https://github.com/ArduPilot/ardupilot_gz#ardupilot_gz)
-> export GZ_VERSION=harmonic
-> source install/setup.bash
-> ros2 launch ardupilot_gz_bringup iris_runway.launch.py
