# Where Am I

```bash
cd project04

cd src
catkin_init_workspace

cd ..
catkin_make
source devel/setup.zsh

# Terminal 1
roslaunch my_robot world.launch
# Terminal 2
roslaunch my_robot mapping.launch
# Terminal 3
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

View Mapping Database
```bash
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

![screenshot](Screenshot.png)