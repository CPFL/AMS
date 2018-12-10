# Autoware-Management-System
Open-source software for the fleet management and simulation system of vehicles powered by Autoware.

[![Autoware Taxi](http://drive.google.com/uc?export=view&id=1liFH9UgKBOVA7zZDmJQhCUcoCYG2eDEs)](https://drive.google.com/file/d/1wLBVNnebWPo6zA8Bcraoo0q2_lMYRAVx/view)

[![Sim Taxi](http://drive.google.com/uc?export=view&id=1j_DvBoJ1yivr1UzUhhRdUCUPadhdL9rf)](https://drive.google.com/open?id=1tFfS3x77OTAZ2zB2cgNi74JiX2px8N__)

## Requirements

- mqtt broker ( ex. mosquitto )
- pipenv

## Installing

Only Python 3.6+ is supported.

```terminal
$ pipenv install --dev
```

## Running

1. Launch Flask server and nodes(fleet_manager, vehicle, user, traffic_signal).

```terminal
exammples $ pipenv install
examples/sim_taxi $ pipenv run python launcher.py
```

2. Access localhost:5000
