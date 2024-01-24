# explainable_ros

## Usage
```shell
$ ros2 launch explicability_bringup explicability_ros.launch.py
```

```shell
$ ros2 service call /question explicability_msgs/srv/Question "{'question': 'What is happening?'}"
```
