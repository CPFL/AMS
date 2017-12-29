message = {
    "name": "signal name",
    "time": "published unixtime",
    "routes": [{
        "route_code": "route_code",
        "action": "next signal state from external: None, green, yellow or red",
        "state": "current signal state: None, green, yellow or red"
    }],
    "schedules": [{
        "route_code": "route_code",
        "state": "next signal state: green, yellow or red",
        "start_time": "start time of next signal state",
        "duration": "duration of next signal state"
    }],
    "cycles":[{
        "route_codes": [],
        "base_time": None,
        "period": None,
        "phases": [{
            "state": "signal state: green, yellow or red",
            "duration": "duration of signal state"
        }]
    }]
}
