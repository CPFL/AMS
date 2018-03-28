let map=null;
let users = {};
const userMarkers = {};
let vehicles = {};
const vehicleMarkers = {};
let trafficSignals = {};
let fleetStatus = {};
const trafficSignalPolylines = {};
const routeMarkers = {};
const waypointCircles = {};
const routes = {};
const circles = {};
const date = new Date() ;
const socket = io.connect("ws://" + window.location.hostname + ":" + window.location.port + "/ams");

const toDegree = 360/(2*Math.PI);
Math.degree = Math.degree || function(rad){
    return rad * toDegree;
}

function onLoad() {
    const xMLHttpRequestViewData = new XMLHttpRequest();
    xMLHttpRequestViewData.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            viewData = JSON.parse(this.responseText);

            socket.addEventListener(viewData.topics.sim_taxi_user, function (mqtt_message) {
                setUser(mqtt_message.topic.split("/")[2], mqtt_message.topic.split("/")[3], mqtt_message.message);
                if(map!=null) {
                    drawTrafficSignals();
                    drawUsers();
                    drawVehicles();
                }
            });

            socket.addEventListener(viewData.topics.sim_taxi, function (mqtt_message) {
                setVehicle(mqtt_message.topic.split("/")[2], mqtt_message.topic.split("/")[3], mqtt_message.message);
                if(map!=null) {
                    drawTrafficSignals()
                    drawRoutes();
                    // drawAutowareWaypoints();
                    drawUsers();
                    drawVehicles();
                }
            });

            socket.addEventListener(viewData.topics.sim_taxi_fleet, function (mqtt_message) {
                setFleetStatus(mqtt_message.topic.split("/")[3], mqtt_message.message);
                if(map!=null) {
                    drawUsers();
                    drawVehicles();
                }
            });

            socket.addEventListener(viewData.topics.sim_bus_user, function (mqtt_message) {
                setUser(mqtt_message.topic.split("/")[2], mqtt_message.topic.split("/")[3], mqtt_message.message);
                if(map!=null) {
                    drawTrafficSignals();
                    drawUsers();
                    drawVehicles();
                }
            });

            socket.addEventListener(viewData.topics.sim_bus, function (mqtt_message) {
                setVehicle(mqtt_message.topic.split("/")[2], mqtt_message.topic.split("/")[3], mqtt_message.message);
                if(map!=null) {
                    drawTrafficSignals()
                    drawRoutes();
                    // drawAutowareWaypoints();
                    drawUsers();
                    drawVehicles();
                }
            });

            socket.addEventListener(viewData.topics.sim_bus_fleet, function (mqtt_message) {
                setFleetStatus(mqtt_message.topic.split("/")[3], mqtt_message.message);
                if(map!=null) {
                    drawUsers();
                    drawVehicles();
                }
            });

            socket.addEventListener(viewData.topics.traffic_signal, function (mqtt_message) {
                setTrafficSignals(mqtt_message.topic.split("/")[3], mqtt_message.message);
                if(map!=null) {
                    drawTrafficSignals();
                    drawRoutes();
                    // drawAutowareWaypoints();
                    drawUsers();
                    drawVehicles();
                }
            });

            initMap();
        }
    };

    xMLHttpRequestViewData.open("GET", "http://" + window.location.hostname + ":" + window.location.port + "/getViewData", true);
    xMLHttpRequestViewData.send();
};

function setFleetStatus(fleetManagerID, message){
    fleetStatus = JSON.parse(message);
}

function setUser(userType, userID, message) {
    const user = JSON.parse(message);
    user.userID = userID;
    user.toID = null;
    user.type = userType;
    switch (user.type) {
        case "sim_taxi_user":
            console.log(user);
            const targetSimTaxi = user.schedule.targets.find(function(element, index, array){
                if(element.group=="SimTaxi"){
                    return true
                }
                return false
            });
            if(typeof targetSimTaxi !== "undefined"){
                // console.log(targetSimBus);
                user.toID = targetSimTaxi.id;
            }

            // if (Object.keys(fleetStatus).includes("relations")) {
            //     if(Object.keys(fleetStatus.relations).includes("SimTaxiUser/"+userID))
            //     {
            //         if(fleetStatus.relations["SimTaxiUser/"+userID].length==1){
            //             user.toID = fleetStatus.relations["SimTaxiUser/"+userID][0].split("SimTaxi/")[1];
            //         }
            //     }
            //     switch (user.state) {
            //         case "waiting":
            //             user.color = "#FF0000";
            //             break;
            //         case "moving":
            //             user.color = "#00a3e0";
            //             break;
            //         case "getting_on":
            //         case "getting_out":
            //             user.color = "#00FF00";
            //             break;
            //         case "login":
            //         default:
            //             user.color = "#000000";
            //             break;
            //     }
            // }
            break;
        case "sim_bus_user":
            const targetSimBus = user.schedule.targets.find(function(element, index, array){
                if(element.group=="SimBus"){
                    return true
                }
                return false
            });
            // console.log(targetSimBus);
            if(typeof targetSimBus !== "undefined"){
                // console.log(targetSimBus);
                user.toID = targetSimBus.id;
            }
            // console.log(user);
            break;
    }

    users[userID] = user;
}

function setVehicle(vehicleType, vehicleID, message) {
    const vehicle = JSON.parse(message);
    vehicle.vehicleID = vehicleID;
    vehicle.type = vehicleType;
    switch (vehicle.type) {
        case "sim_taxi":
            vehicle.path = "M29.395,0H17.636c-3.117,0-5.643,3.467-5.643,6.584v34.804c0,3.116,2.526,5.644,5.643,5.644h11.759 c3.116,0,5.644-2.527,5.644-5.644V6.584C35.037,3.467,32.511,0,29.395,0z M34.05,14.188v11.665l-2.729,0.351v-4.806L34.05,14.188z M32.618,10.773c-1.016,3.9-2.219,8.51-2.219,8.51H16.631l-2.222-8.51C14.41,10.773,23.293,7.755,32.618,10.773z M15.741,21.713 v4.492l-2.73-0.349V14.502L15.741,21.713z M13.011,37.938V27.579l2.73,0.343v8.196L13.011,37.938z M14.568,40.882l2.218-3.336 h13.771l2.219,3.336H14.568z M31.321,35.805v-7.872l2.729-0.355v10.048L31.321,35.805z";
            vehicle.anchor = new google.maps.Point(24, 25);
            switch (vehicle.state) {
                case "move":
                case "move_for_picking_up":
                case "move_for_discharging":
                    vehicle.color = "#00a3e0";
                    break;
                case "stop":
                case "stop_for_picking_up":
                case "stop_for_discharging":
                    vehicle.color = "#FF0000";
                    break;
                case "moveToDeploy":
                default:
                    vehicle.color = "#000000";
                    break;
            }
            break;
        case "sim_bus":
            vehicle.path = "M25,5,Q 25 0 30 0,H45,Q50 0 50 5,V45,Q50 50 45 50,H30,Q25 50 25 45,Z";
            vehicle.anchor = new google.maps.Point(34.5,25);
            switch (vehicle.state) {
                case "move":
                case "move_to_circular_route":
                case "move_to_select_point":
                case "move_to_bus_stop":
                case "move_to_junction":
                    vehicle.color = "#00a3e0";
                    break;
                case "move_to_branch_point":
                case "request_via_schedules":
                case "request_through_schedules":
                    vehicle.color = "#FF9900";
                    break;
                case "stop":
                case "stop_for_taking_up":
                case "stop_for_discharging":
                case "stop_for_discharging_and_taking_up":
                    vehicle.color = "#FF0000";
                    break;
                case "move_to_parking":
                case "stop_to_parking":
                default:
                    vehicle.color = "#000000";
                    break;
            }
            break;
    }
    vehicles[vehicleID] = vehicle;
}

function setTrafficSignals(trafficSignalID, message) {
    trafficSignals[trafficSignalID] = JSON.parse(message);
}

function drawUsers() {
    for(const key in users) {
        const waypoint = viewData.waypoints[users[key].trip_schedules[0].route.start_waypoint_id];
        let latLng = geohashToLatLng(waypoint.geohash);
        const goalWaypoint = viewData.waypoints[users[key].trip_schedules[0].route.goal_waypoint_id];
        const goalLatLng = geohashToLatLng(goalWaypoint.geohash);

        if(waypoint === undefined) { continue; }
        switch (users[key].type) {
            case "sim_taxi_user":
                if (users[key].toID != null && ["getting_on", "got_on", "moving", "getting_out"].includes(users[key].state)) {
                    latLng = geohashToLatLng(vehicles[users[key].toID].location.geohash);
                }
                break;
            case "sim_bus_user":
                if (users[key].toID != null && ["getting_on", "got_on", "moving", "ready_to_get_out", "getting_out"].includes(users[key].state)) {
                    latLng = geohashToLatLng(vehicles[users[key].toID].location.geohash);
                }
                break;
        }

        if (["got_out", "log_out"].includes(users[key].state)) {
            if (key in userMarkers) {
                userMarkers[key]["icon"].setMap(null);
                userMarkers[key]["destination"].setMap(null);
                delete userMarkers[key];
            }
            delete users[key];
            continue;
        }

        if (key in userMarkers) {
            userMarkers[key]["icon"].setPosition( latLng );
            userMarkers[key]["icon"].setIcon({
                url: "http://download.seaicons.com/icons/custom-icon-design/mono-business/24/user-icon.png",
                size: new google.maps.Size(20, 32),
                origin: new google.maps.Point(0, 0),
                anchor: new google.maps.Point(0, 32)
            });
        }
        else {
            userMarkers[key] = {
                "icon": new google.maps.Marker({
                    label: users[key].name,
                    position: latLng,
                    map: map,
                    draggable: false,
                    icon: {
                    url: "http://download.seaicons.com/icons/custom-icon-design/mono-business/24/user-icon.png",
                    // This marker is 20 pixels wide by 32 pixels high.
                    size: new google.maps.Size(20, 32),
                    // The origin for this image is (0, 0).
                    origin: new google.maps.Point(0, 0),
                    // The anchor for this image is the base of the flagpole at (0, 32).
                    anchor: new google.maps.Point(0, 32)
                    }
                }),
                "destination": new google.maps.Polyline({
                    path: [latLng, goalLatLng],
                    icons: [{
                        icon: {
                            path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
                        },
                        offset: '100%'
                    }],
                    //geodesic: true,
                    strokeColor: '#000000',
                    strokeOpacity: 0.5,
                    strokeWeight: 4,
                    map: map
                })
            };
        }
    }
}

function drawTrafficSignal(routeCode, color) {
    const route = getRouteFromRouteCode(routeCode);
    trafficSignalPolylines[routeCode] = drawRoute(route, color, strokeWidth=5, strokeOpacity=1.0);
}

function drawTrafficSignals() {
    for(const trafficSignalID in trafficSignals) {
        if (trafficSignals[trafficSignalID].route_code in trafficSignalPolylines) {
            trafficSignalPolylines[trafficSignals[trafficSignalID].route_code].setMap(null);
        }
    }
    for(const trafficSignalID in trafficSignals) {
        if(trafficSignals[trafficSignalID].state=="red") {
            drawTrafficSignal(trafficSignals[trafficSignalID].route_code, "#FF0000");
        }
        else if(trafficSignals[trafficSignalID].state=="unknown") {
            drawTrafficSignal(trafficSignals[trafficSignalID].route_code, "#FF0000");
        }
    }
    for(const trafficSignalID in trafficSignals) {
        if(trafficSignals[trafficSignalID].state=="yellow") {
            drawTrafficSignal(trafficSignals[trafficSignalID].route_code, "#FFFF00");
        }
    }
    for(const trafficSignalID in trafficSignals) {
        if(trafficSignals[trafficSignalID].state=="green") {
            drawTrafficSignal(trafficSignals[trafficSignalID].route_code, "#00FF00");
        }
    }
}

function geohashToLatLng(geohash) {
    const latLng3 = decodeGeoHash(geohash);
    return {"lat": latLng3.latitude[2], "lng": latLng3.longitude[2]};
}

function drawVehicles() {
    for(const key in vehicles) {
        const latLng = new google.maps.LatLng( geohashToLatLng(vehicles[key].location.geohash) );
        if( key in vehicleMarkers ) {
            vehicleMarkers[key].setPosition(latLng);
            vehicleMarkers[key].setLabel(vehicles[key].name + "@" + vehicles[key].state);
            vehicleMarkers[key].setIcon({
                path: vehicles[key].path,
                fillColor: vehicles[key].color,
                fillOpacity: .6,
                anchor: vehicles[key].anchor,
                strokeWeight: 0,
                rotation: Math.degree(vehicles[key].pose.orientation.rpy.yaw),
                scale: 1
            });
        }
        else {
            vehicleMarkers[key] = new google.maps.Marker({
                label: vehicles[key].name+"@"+vehicles[key].state,
                position: latLng,
                map: map,
                draggable: false,
                icon: {
                    path: vehicles[key].path,
                    fillColor: vehicles[key].color,
                    fillOpacity: .6,
                    anchor: vehicles[key].anchor,
                    strokeWeight: 0,
                    rotation: vehicles[key].pose.orientation.rpy.yaw,
                    scale: 1
                }
            });
        }
    }
}

function drawRoute(route, strokeColor="#00FF00", strokeWeight=4, strokeOpacity=0.5) {
    const paths = getRoutePaths(route);

    return new google.maps.Polyline({
        path: paths,
        icons: [{
            icon: {
                path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
            },
            offset: '100%'
        }],
        //geodesic: true,
        strokeColor: strokeColor,
        strokeOpacity: strokeOpacity,
        strokeWeight: strokeWeight,
        map: map
    });
}

function drawRoutes() {
    for(const vehicleID in vehicles) {
        if (vehicleID in routeMarkers) {
            routeMarkers[vehicleID].setMap(null);
        }
        if(["move", "move_for_picking_up", "move_for_discharging"].includes(vehicles[vehicleID].state)) {
            routeMarkers[vehicleID] = drawRoute(vehicles[vehicleID].schedule.route, strokeColor="#00FFFF", strokeWeight=4);
        }
        if(["move", "move_to_circular_route", "move_to_select_point", "move_to_junction"].includes(vehicles[vehicleID].state)) {
            routeMarkers[vehicleID] = drawRoute(vehicles[vehicleID].schedule.route, strokeColor="#00FFFF", strokeWeight=4);
        }
        if(["move_to_branch_point"].includes(vehicles[vehicleID].state)) {
            routeMarkers[vehicleID] = drawRoute(vehicles[vehicleID].schedule.route, strokeColor="#FF9900", strokeWeight=4);
        }
        if(["move_to_bus_stop"].includes(vehicles[vehicleID].state)) {
            routeMarkers[vehicleID] = drawRoute(vehicles[vehicleID].schedule.route, strokeColor="#FF0000", strokeWeight=4);
        }
    }
}

function drawAutowareWaypoints() {
    for(const vehicleID in vehicles) {
        if (vehicleID in waypointCircles) {
            for(const key in waypointCircles[vehicleID]) {
                waypointCircles[vehicleID][key].setMap(null);
            }
        }
        waypointCircles[vehicleID] = {}
        if(["move", "moveToUser", "moveToUserDestination"].includes(vehicles[vehicleID].state)) {
            const arrowCodes = vehicles[vehicleID].schedule["route"]["arrow_codes"];
            const startWaypointID = vehicles[vehicleID].waypoint_id;
            const goalWaypointID = vehicles[vehicleID].schedule["route"].goal_waypoint_id;

            let centers = [];
            const ie = Math.min(2, arrowCodes.length);
            for(let i=0; i<ie; i++) {
                const arrowCode = arrowCodes[i];
                let js = 0
                if(viewData.arrows[arrowCode].waypointIDs.includes(startWaypointID)){
                    js = viewData.arrows[arrowCode].waypointIDs.indexOf(startWaypointID)
                }
                let je = viewData.arrows[arrowCode].waypointIDs.length;
                if(viewData.arrows[arrowCode].waypointIDs.includes(goalWaypointID)){
                    je = viewData.arrows[arrowCode].waypointIDs.indexOf(goalWaypointID)
                }
                for(let j=js; j<je; j++) {
                    const waypoint_id = viewData.arrows[arrowCode].waypointIDs[j];
                    centers.push(geohashToLatLng(viewData.waypoints[waypoint_id].geohash));
                }
            }
            for(const key in centers) {
                waypointCircles[vehicleID][key] = new google.maps.Circle({
                    strokeColor: vehicles[vehicleID].color,
                    strokeOpacity: 0.8,
                    strokeWeight: 2,
                    fillColor: vehicles[vehicleID].color,
                    fillOpacity: 0.8,
                    map: map,
                    center: centers[key],
                    radius: 0.2
                });
            }
        }
    }
}

function getRouteFromRouteCode(routeCode) {
    const splittedRouteCode = routeCode.split(":");
    const splittedRouteCodeMiddles = splittedRouteCode[1].split("_");
    let arrowCodes = [];
    for( let i=1; i<splittedRouteCodeMiddles.length; i++ ){
        arrowCodes.push(splittedRouteCodeMiddles[i-1]+"_"+splittedRouteCodeMiddles[i])
    }
    route = {
        start_waypoint_id: splittedRouteCode[0],
        goal_waypoint_id: splittedRouteCode[2],
        arrow_codes: arrowCodes,
    }
    return route
}

function getRoutePaths(route) {
    const arrowCodes = route.arrow_codes;
    const startWaypointID = route.start_waypoint_id;
    const goalWaypointID = route.goal_waypoint_id;
    let paths = [];
    for(const i in arrowCodes) {
        const arrowCode = arrowCodes[i];
        let js = 0
        if(viewData.arrows[arrowCode].waypointIDs.includes(startWaypointID)) {
            js = viewData.arrows[arrowCode].waypointIDs.indexOf(startWaypointID)
        }
        let je = viewData.arrows[arrowCode].waypointIDs.length;
        if(viewData.arrows[arrowCode].waypointIDs.includes(goalWaypointID)) {
            je = viewData.arrows[arrowCode].waypointIDs.indexOf(goalWaypointID) + 1
        }
        for(let j=js; j<je; j++) {
            const waypoint_id = viewData.arrows[arrowCode].waypointIDs[j];
            paths.push(geohashToLatLng(viewData.waypoints[waypoint_id].geohash));
        }
    }
    return paths;
}

function drawArrow(arrowCode, strokeColor="#000000", strokeWidth=2, dLatLng=0) {
    const splittedArrowID = arrowCode.split("_");

    let paths = getArrowPaths(arrowCode)

    return new google.maps.Polyline({
        path: paths,
        icons: [{
            icon: {
                path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
            },
            offset: '100%'
        }],
        //geodesic: true,
        strokeColor: strokeColor,
        strokeOpacity: 0.3,
        strokeWeight: strokeWidth,
        map: map
    });
}

function drawArrows() {
    const arrows = viewData.arrows;
    for(const key in arrows) {
        let path = [];
        for(const waypoint_id of arrows[key].waypointIDs) {
            path.push(geohashToLatLng(viewData.waypoints[waypoint_id].geohash));
        }

        var arrow = new google.maps.Polyline({
            path: path,
            shapes: arrows[key].waypointIDs,
            icons: [{
                icon: {
                    path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
                },
                offset: '100%'
            }],
            //geodesic: true,
            strokeColor: '#A9A9A9',
            strokeOpacity: 0.3,
            //strokeWeight: 2,
            map: map
        });
        arrow.addListener('click', function() {
            console.log(this.shapes);
        });
    }
}

function initMap() {
    console.log("initMap");
    map = new google.maps.Map(document.getElementById('map'), {
        zoom: 19,
        center: {lat: parseFloat(viewData.viewPoint.lat), lng: parseFloat(viewData.viewPoint.lng)},
        mapTypeId: 'terrain'
    });

    console.log(viewData);

    drawArrows();
    const xhttpFleetStatus = new XMLHttpRequest();
    xhttpFleetStatus.open("GET", "http://" + window.location.hostname + ":" + window.location.port + "/requestFleetRelations", true);
    xhttpFleetStatus.send();
}