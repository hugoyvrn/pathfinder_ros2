# pathfinder_ros2
ROS2 driver for a serie connection with the PathFinder DVL.


TODO @HG



###  Install rtac_asio
You need to clone the `rtac_asio` git and follow the README.md. Go to https://github.com/ENSTABretagneRobotics/rtac_asio.

In your install directory in `<your install location>`(for exemple: `~/work/my_user`) clone the repository:
```bash
git clone https://github.com/ENSTABretagneRobotics/rtac_asio.git
cd rtac_asio
git switch master # ?
```
handle the git branch as you want. Please keep in mind you have to pull the reposetory yourself to keep up to date.

Create a build directory in the root of the reposetory :
```bash
mkdir build && cd build
```
Make sure the CMAKE_PREFIX_PATH environment variable contains your install
location :
```bash
echo $CMAKE_PREFIX_PATH
```

If not, put this at the end your `$HOME/.bashrc` file:
```bash
export CMAKE_PREFIX_PATH=<your install location>:$CMAKE_PREFIX_PATH
```
and source again your bashrc.

Generate your build system with CMake, compile and install :
```bash
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<your install location> ..
make -j4 install
```

### Build
Then build the packages of the project:
```bash
colcon build --symlink-install --packages-select data oculus_ros2 oculus_interfaces description slamac sonar_to_octomap
```

---