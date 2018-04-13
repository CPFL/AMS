# How to connect Autoware(ROS) to AMS(MQTT)

### Setting up in Autoware installed PC.

1. Install [ros-mqtt-bridge](https://github.com/CPFL/ros_mqtt_bridge).

```console
$ pip install -e git+https://github.com/CPFL/ros_mqtt_bridge.git@v0.1#egg=ros_mqtt_bridge
```

2. Setting ip and port for mqtt broker.

```console
ams/example/ros/config $ vim sample.env

  3 MQTT_BROKER_HOST="your mqtt-broker's host"
  4 MQTT_BROKER_PORT="your mqtt-broker's port"

```

3. Launch the bridges.

```console
ams/example/ros $ python launch.py
```
