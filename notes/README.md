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

