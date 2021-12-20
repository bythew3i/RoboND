# Build My World

```bash
mkdir build
cd build

cmake ..
make

export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${PWD}

cd ..
gazebo world/myworld
```