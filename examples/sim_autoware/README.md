- install ams

- install mosquitto and edit config

``` console
 mac $ vim /usr/local/etc/mosquitto/mosquitto.conf
 ubuntu $ vim /etc/mosquitto/mosquitto.conf
```

add next three lines
```
listener 1883
listener 8080 127.0.0.1
protocol websockets
```

- install requirements

``` console
 ams/examples/sim_autoware $ pip install -r requirements.txt
```

- launch flask server

``` console
 ams/examples/sim_autoware $ python viewer.py
```

- launch browser and access

```
localhost:5000
```

