# How to connect Autoware(ROS) to AMS(MQTT)

### Setting up in Autoware installed PC.

1. Install [ros-mqtt-bridge](https://github.com/CPFL/ros_mqtt_bridge).

2. Setting endpoint for AWS IoT.

```console
ams/example/ros_aws_iot/config $ vim sample.env

  1 AWS_IOT_ENDPOINT="**************.iot.ap-northeast-1.amazonaws.com"

```

3. Setting credential files for AWS IoT.

4. Launch the bridges.

```console
ams/example/ros_aws_iot $ python launch.py -CAP "your/root_ca/file/path" -KP "your/private_key/file/path" -CP "your/certificate/file_path"
```
