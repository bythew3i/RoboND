# Robotics Software Engineer

[toc]

## Gazebo 

> `gazebo` = `gzserver` + `gzclient`

### Gazebo Server

- Responsible for parsing the description file related to scene/objects
- Can launch independently by `gzserver`

### Gazebo Client

- Provide graphic client that connects to **gzserver**
- Render the simulation scene
- Can run independently by `gzclient` (waste if no gzserver)



### World

```bash
gazebo <yourworld>.world
```

Formatted in SDF (Simulation Description Format):

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <physics type="ode">
      ...
    </physics>

    <scene>
      ...
    </scene>

    <model name="box">
      ...
    </model>

    <model name="sphere">
      ...
    </model>

    <light name="spotlight">
      ...
    </light>

  </world>
</sdf>
```



### Model

Include model in world

```xml
<include>
  <uri>model://model_file_name</uri>
</include>
```



### Env Var

eg: `GAZEBO_MODEL_PATH`



### Plugin

- Interactive with the world
- Can be loaded from cmd or added to world SDF

Example:

hello.cpp

```cpp
#include <gazebo/gazebo.hh>

namespace gazebo {
    class WorldPluginMyRobot : public WorldPlugin {
      public:
        // Constructor
        WorldPluginMyRobot() : WorldPlugin() {
            printf("Hello World!\n");
        }
        
        // Mandatory
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {

        }
    
    };

    GZ_REGISTER_WORLD_PLUGIN(WorldPluginMyRobot)
}
```

CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(hello SHARED script/hello.cpp)
target_link_libraries(hello ${GAZEBO_LIBRARIES})
```

Export Plugin Path (optional)

```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/workspace/myrobot/build
```

```xml
...
<world name="default">
    <plugin name="hello" filename="libhello.so"/>
...
```



## RViz

- stands for ROS Visualization (Gazebo is a physics simulator)
- can visualize any type of sensor data (can be live-stream or bagfile)

```bash
roscore
```

```bash
# run rviz node in rviz pkg
# rosrun package_name executable_name
rosrun rviz rivz
```





## ROS

### History

- **2000s** Stanford AI Lab
- **2007** Willow Garage startup
- **2013** Open Source Robotics Foundation



### Topic vs. Service

#### Topic

- massage passing through **subscriber / publisher**

  - define mesaage type in `.msg` file, i.e.: /msg/xxx,msg

    ```
    fieldtype1 fieldname1
    fieldtype2 fieldname2
    fieldtype3 fieldname3
    ```

  - Publisher C++ format

    ```cpp
    // queue_size: can store msg until being sent; exceeding will drop oldest
    ros::Publisher pub1 = n.advertise<message_type>("/topic_name", queue_size);
    ```
  
  - Subscriber C++ format
  
    ```cpp
    ros::Subscriber sub1 = n.subscribe("/topic_name", queue_size, callback_function);
    ```
  
    

#### Service

- **Service**: massage passing through **request / response**

  - define message type in `.srv` file, i.e: /srv/xxx.srv

    ```
    #request constants
    int8 FOO=1
    int8 BAR=2
    #request fields
    int8 foobar
    another_pkg/AnotherMessage msg
    ---
    #response constants
    uint32 SECRET=123456
    #response fields
    another_pkg/YetAnotherMessage val
    CustomMessageDefinedInThisPackage value
    uint32 an_integer
    ```

  - Server c++ format

    ```cpp
    ros::ServiceServer service = n.advertiseService(`service_name`, handler);
    ```
    
  - Client c++ format
  
    ```c++
    ros::ServiceClient client = n.serviceClient<package_name::service_file_name>("service_name");
    ```

  - Client call Server on Command Line

    ```bash
    rosservice call service_name “request”
    ```

  
  




### Turtlesim Example

#### Start nodes

```bash
# Master Process
roscore
```

```bash
rosrun turtlesim turtlesim_node
```

```bash
rosrun turtlesim turtle_teleop_key
```

#### List all active nodes

```bash
rosnode list
```

- `/rosout`: launched by roscore. It subscribes to the standard **/rosout topic** which all nodes send log messages.
- `/teleop_turtle`: keyboard teleop node
- `/turtlesim`: node name for turtlebot_sim node

#### List all topic

```bash
rostopic list
```

- `/rosout`
- `/rosout_agg`: aggregated feed of messages

- `/turtle1/cmd_vel`: velosity cmds

- `/turtle1/color_sensor`: readings from the sensor are published to this topic

- `/turtle1/pose`: pose and orientation of turtle are published to this topic



#### Get topic info

```
rostopic info /turtle1/cmd_vel
```

- Type: geometry_msgs/Twist

- Publishers: 
   * /teleop_turtle (http://jinux:38441/)


- Subscribers: 
   * /turtlesim (http://jinux:38975/)



#### Get message info

```bash
rosmsg info geometry_msgs/Twist
```

```
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

Edit ros msg

```bash
rosed geometry_msgs Twist.msg
```

```bash
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
```



#### Echo message on a topic

Listen to a topic

```bash
rostopic echo /turtle1/cmd_vel
```



### URDF

- Unified Robot Description Format

- XML markup language

```xml
<?xml version="1.0"?>
<robot name="two_link_robot">
  <!--Links-->
  <link name="link_1">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.2"/>
      </geometry>
    </visual>
  </link>
  <link name="link_2">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
    </visual>
  </link>
  <!--Joints-->
  <joint name="joint_1" type="continuous">
    <parent link="link_1"/>
    <child link="link_2"/>
  </joint>
</robot>
```



## Catkin

### Workspace

A top-level directory where you build, install and modify catkin packages

Create a package:

Step 1: Create a catkin workspace and a sub directory

```bash
mkdir -p ~/workspace/catkin_ws/src
```

Step 2: Navigate to source directory

```bash
cd ~/workspace/catkin_ws/src
```

Step 3: Initialize the catkin workspace

```bash
catkin_init_workspace
```

Step 4: Build

```bash
cd ..
catkin_make
```

### Package

A directory might contain

- CMakeLists.txt
- package.xml 
- source code
- libraries
- database
- more ...

Clone simple_arm example:

```bash
cd ~/workspace/catkin_ws/src/
git clone -b first_interaction https://github.com/udacity/RoboND-simple_arm/ simple_arm
```

Build the simple_arm package

```bash
cd ..
catkin_make
```



Create a catkin package:

```bash
cd ~/workspace/catkin_ws/src
# catkin_create_pkg <your_package_name> [dependency1 dependency2 …]
catkin_create_pkg first_package
```





### Roslaunch

- launch ROS **Master and multiple** nodes
- Set default parameter on parameter server
- Auto re-spawn processes that have died

```bash
cd ~/workspace/catkin_ws/
catkin_make

# source devel/setup.zsh if you are in zsh
source devel/setup.bash

roslaunch simple_arm robot_spawn.launch
```



### Rosdep

- check package's missing dependencies
- download and install them

```bash
# must be sourced with source devel/setup.bash
rosdep check <package name>

rosdep install -i <package name>
```



