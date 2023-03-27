# nav2_accountability_explainability
This repository includes a ROS 2 accountability and explainability solution based on the use of its topics. This approach aims to identify the causes that have triggered a set of specific events, providing to the final user a non-expert explanation.


# Software artifacts
ROS 2 Humble

RB1 simulator for ROS 2

[AWS RoboMaker Hospital](https://github.com/aws-robotics/aws-robomaker-hospital-world) World ROS package available [here](https://github.com/jmguerreroh/aws-robomaker-hospital-world/tree/ros2).

The following images show the floor plan of the scenario used in the development of this work, including in the second one an obstacle in the route initially calculated to reach the goal destination. This obstacle will force the change of the pre-computed path.

![imagen](https://user-images.githubusercontent.com/13176052/227868761-7df42f3d-9043-4b07-af27-2b843806be0e.png)

![imagen](https://user-images.githubusercontent.com/13176052/227868841-21b6f0e0-1017-4136-94aa-396ba1205a6b.png)


# Usage
## Accountability solution
After starting the simulation, black-box recording information service can be started with the command
```
ros2 launch bag_recorder bag_recorder.launch.py 
```
The previous recording service can be started and stopped by running
```
ros2 service call /bag_recorder std_srvs/srv/SetBool data:\ true
ros2 service call /bag_recorder std_srvs/srv/SetBool data:\ false
```
To start the recording process only when it would be an obstacle, it must be run
```
ros2 run bag_recorder bag_recorder_scan_client
```
To start the recording process only when a nav2 behavior tree node would be in FAILURE status, it must be run
```
ros2 run bag_recorder bag_recorder_btstatus_client
```
The navigation goal pose, can be sent by running
```
ros2 run goal_sender nav_to_pose_action_client 
```
The reproducibility of the recorded information can be tested with the command
```
ros2 bag play <path to rosbag file>
```
A sample of the recorded data can be found [here](https://drive.google.com/drive/folders/19dJu8P5nJbYrA8s7bAF5fEMyJEJk0LYS?usp=sharing).
## Explainability solution
After starting the simulation, explainability service can be started with the command
```
ros2 launch navigation_explainer navigation_explainer_srv.launch.py 
```
Explanations can be obtained by running
```
ros2 run navigation_explainer navigation_explainer_client "What is the current navigation status?"
ros2 run navigation_explainer navigation_explainer_client "Why have you changed the planned path?"
```
### Output example
#### Explanations obtained before sending a goal pose
Next is shown the explanation service's output in the initial navigation state where no goal pose has been sent. A graphical representation of this state in the Rviz2 tool is displayed in the following image.
```
ros2 run navigation_explainer navigation_explainer_client "What is the current navigation status?"
[INFO] [1679254837.887102801] [navigation_explainer_client]: 'No navigation is running'
```
```
ros2 run navigation_explainer navigation_explainer_client "Why have you changed the planned path?"
[INFO] [1679254848.061037165] [navigation_explainer_client]: 'I have not changed the planned path'
```
![imagen](https://user-images.githubusercontent.com/13176052/227803431-e6308617-7b7f-41b1-8898-f87ffc4decc9.png)
#### Explanations obtained while navigation is in progress
Next is shown the explanation service's output when sending a goal pose, and the navigation is in progress. A graphical representation of this state in the Rviz2 tool is displayed in the following image.
```
ros2 run navigation_explainer navigation_explainer_client "What is the current navigation status?"
[INFO] [1679255510.444170839] [navigation_explainer_client]: 'Navigation to the goal is in progress.'
```
```
ros2 run navigation_explainer navigation_explainer_client Why have you changed the planned path?"
[INFO] [1679255502.212077103] [navigation_explainer_client]: 'I have not changed the planned path'
```
![imagen](https://user-images.githubusercontent.com/13176052/227803896-0fbaef68-03c9-4e2a-95c7-fe76824b8b16.png)

#### Explanations obtained when an obstacle is detected
Next is shown the explanation service's output when an obstacle is detected. A graphical representation of this state in the Rviz2 tool is displayed in the following images.
```
ros2 run navigation_explainer navigation_explainer_client "What is the current navigation status?"
[INFO] [1679255690.249576203] [navigation_explainer_client]: 'Navigation to the goal is in progress.'
```
```
ros2 run navigation_explainer navigation_explainer_client "Why have you changed the planned path?"
[INFO] [1679255693.275863200] [navigation_explainer_client]: 'I have changed the planned path because there was an obstacle. Then, I followed a new path to the goal pose.'
```
![imagen](https://user-images.githubusercontent.com/13176052/227804154-6458c2c8-763b-4eeb-817b-833589642b54.png)
![imagen](https://user-images.githubusercontent.com/13176052/227804164-977f7085-abec-49df-881a-7e0fbaa972c9.png)

#### Explanations obtained when the navigation ends
Next is shown the explanation service's output when the navigation has successfully ended. A graphical representation of this state in the Rviz2 tool is displayed in the following image.
```
ros2 run navigation_explainer navigation_explainer_client "What is the current navigation status?"
[INFO] [1679255808.341188660] [navigation_explainer_client]: 'Navigation to the goal has succeeded.'
```
```
ros2 run navigation_explainer navigation_explainer_client "Why have you changed the planned path?"
[INFO] [1679255805.321852201] [navigation_explainer_client]: 'I have changed the planned path because there was an obstacle. Then, I followed a new path to the goal pose.'
```
![imagen](https://user-images.githubusercontent.com/13176052/227804518-3a7b1448-8540-40d6-a6b7-d7ce1bda1363.png)
