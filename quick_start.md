If you have all the code setup 

Raspi end:
1. cd to the ros2_localization 
2.
``` 
. install/setup.bash
```
3. 
```
 ros2 launch src\localization\launch\raspi_nodes.launch.py
```
Laptop end:
1. cd to the ros2_localization 
2.
``` 
. install/setup.bash
```
3. 
```
 ros2 launch src\localization\launch\remote_nodes.launch.py
```
4. open another terminal
5. 
```
ros2 run rviz2 rviz2
```