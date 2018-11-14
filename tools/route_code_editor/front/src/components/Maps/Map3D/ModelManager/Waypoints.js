import * as THREE from "three";


/**
 * ADASMapLoader
 */
export default class Object3DManager extends THREE.Group {

    constructor(camera, controls) {
        super();
        this.waypointsList = {};
        this.camera = camera;
        this.controls = controls;

        this.idList = {
            milee: [],
            logiee: [],
            postee: []
        }

    }


    /*
    setSchedules(vehicleId, schedules) {
        if (schedules.length === 0) {
            for (let id in this.sceneData.waypointsList) {
                if (this.sceneData.waypointsList.hasOwnProperty(id)) {
                    for (const meshName in this.sceneData.waypointsList[id].arrow) {
                        if (this.sceneData.waypointsList[id].arrow.hasOwnProperty(meshName)) {
                            const threeJSObject = this.scene.getObjectByName(meshName);
                            if (typeof threeJSObject !== "undefined") {
                                this.scene.remove(threeJSObject);
                            }
                            delete this.sceneData.waypointsList[id].arrow[meshName].threeJSObject;
                        }
                    }
                    delete this.sceneData.waypointsList[id];
                }
            }
        } else {
            if (!this.sceneData.waypointsList.hasOwnProperty(vehicleId)) {
                this.sceneData.waypointsList[vehicleId] = {
                    arrow: {}
                };
            }

            for (let schedule of schedules) {
                if (schedule.poseList.length > 0) {
                    this.controls.target.set(schedules[0].poseList[0].x, schedules[0].poseList[0].y, schedules[0].poseList[0].z);
                    this.setWaypoints(vehicleId, schedule.poseList);
                }
            }

        }
    }


    setWaypoints(vehicleId, waypoints) {
        for (let i = 1; i < waypoints.length; i++) {
            const pos_1 = waypoints[i - 1];
            const pos = waypoints[i];
            let from = new THREE.Vector3(pos_1.x, pos_1.y, pos_1.z);
            let to = new THREE.Vector3(pos.x, pos.y, pos.z);
            const mesh = this.drawArrowHead(0xFF0000, [from, to]);
            mesh.name = "arrow_" + mesh.uuid;
            console.log(mesh.name);
            this.sceneData.waypointsList[vehicleId].arrow[mesh.name] = {
                threeJSObject: mesh
            };
            this.add(this.sceneData.waypointsList[this.centerVehicleId].arrow[mesh.name].threeJSObject);
        }
    }
    */

    setWaypointsList(waypointsSet) {
        let color = {
            milee: "#10c9ff",
            logiee: "#ffffff",
            postee: "#FF0000",
        };


        if (Object.keys(waypointsSet).length > 0) {
            for (const key in waypointsSet) {
                if (waypointsSet.hasOwnProperty(key)) {
                    for (const waypoints of waypointsSet[key]) {
                        if (waypoints.length > 0) {
                            this.controls.target.set(waypoints[0].pose.x, waypoints[0].pose.y, waypoints[0].pose.z);

                            for (let i = 1; i < waypoints.length; i++) {
                                const pos_1 = waypoints[i - 1].pose;
                                const pos = waypoints[i].pose;
                                let from = new THREE.Vector3(pos_1.x, pos_1.y, pos_1.z);
                                let to = new THREE.Vector3(pos.x, pos.y, pos.z);
                                const arrow_mesh = this.drawArrowHead(color[key], [from, to]);
                                arrow_mesh.name = "arrow_" + arrow_mesh.uuid;
                                this.waypointsList[arrow_mesh.name] = arrow_mesh;
                                this.add(this.waypointsList[arrow_mesh.name]);
                                this.idList[key].push(arrow_mesh.name);

                            }
                        }
                    }
                }
            }
        }else {
            for (const meshName in this.waypointsList) {
                if (this.waypointsList.hasOwnProperty(meshName)) {
                    const threeJSObject = this.getObjectByName(meshName);
                    if (typeof threeJSObject !== "undefined") {
                        this.remove(threeJSObject);
                    }
                    delete this.waypointsList[meshName].threeJSObject;
                }
            }
            delete this.waypointsList;
            this.waypointsList = {};
            this.idList = {
                milee: [],
                logiee: [],
                postee: []
            };
        }
    }

    emphasisLine(vehicleId, vehicleInfo){

        let key = vehicleId === "ms1" ? "milee" : (vehicleId === "ls1" ? "logiee" : "postee" );
        let nearColor = {
            milee: "#10c9ff",
            logiee: "#ffffff",
            postee: "#FF0000",
        };

        let color = {
            milee: "#2d3ddd",
            logiee: "#b9b8bc",
            postee: "#8b0000",
        };
        for(const waypointId of this.idList[key]){
            const waypointArrow = this.getObjectByName(waypointId);
            let distance = Math.sqrt((waypointArrow.position.x - vehicleInfo.pose.position.x) * (waypointArrow.position.x - vehicleInfo.pose.position.x) +
                (waypointArrow.position.y - vehicleInfo.pose.position.y) * (waypointArrow.position.y- vehicleInfo.pose.position.y) +
                (waypointArrow.position.z - vehicleInfo.pose.position.z) * (waypointArrow.position.z - vehicleInfo.pose.position.z));

            if(distance < 5.0){
                waypointArrow.setColor(new THREE.Color(nearColor[key]));
            }else{
                waypointArrow.setColor(new THREE.Color(color[key]));
            }
        }
    }



    setBinaryWaypoints(waypoints) {


        for (const meshName in this.waypointsList) {
            if (this.waypointsList.hasOwnProperty(meshName)) {
                const threeJSObject = this.getObjectByName(meshName);
                if (typeof threeJSObject !== "undefined") {
                    this.remove(threeJSObject);
                }
                delete this.waypointsList[meshName].threeJSObject;
            }
        }
        delete this.waypointsList;

        if (waypoints.length > 0) {
            this.waypointsList = {};

            this.controls.target.set(waypoints[0].pose.x, waypoints[0].pose.y, waypoints[0].pose.z);

            for (let i = 1; i < waypoints.length; i++) {
                const pos_1 = waypoints[i - 1].pose;
                const pos = waypoints[i].pose;
                let from = new THREE.Vector3(pos_1.x, pos_1.y, pos_1.z);
                let to = new THREE.Vector3(pos.x, pos.y, pos.z);
                const arrow_mesh = this.drawArrowHead(0xFF0000, [from, to]);
                arrow_mesh.name = "arrow_" + arrow_mesh.uuid;
                this.waypointsList[arrow_mesh.name] = arrow_mesh;
                this.add(this.waypointsList[arrow_mesh.name]);
            }
        }
    }

    drawLine(color, points) {
        let material = new THREE.LineBasicMaterial({color: color});
        let geometry = new THREE.Geometry();
        for (const point of points) {
            geometry.vertices.push(new THREE.Vector3(point.x, point.y, point.z));
        }
        geometry.computeBoundingSphere();
        const line = new THREE.Line(geometry, material);
        return line;
    }

    drawArrowHead(color, points) {
        let from = new THREE.Vector3(points[0].x, points[0].y, points[0].z);
        let to = new THREE.Vector3(points[1].x, points[1].y, points[1].z);
        let direction = to.clone().sub(from);
        let length = direction.length();
        return new THREE.ArrowHelper(direction.normalize(), from, length, color, 0.5 * length, 0.3 * length);
    }

    drawCylinder(color, pose, scale) {
        let geometry = new THREE.CylinderGeometry(0.5 * scale.x, 0.5 * scale.y, scale.z, 16, 1, false);
        let material = new THREE.MeshBasicMaterial({color: color});
        let mesh = new THREE.Mesh(geometry, material);
        mesh.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI * 0.5);
        mesh.position.x = pose.position.x;
        mesh.position.y = pose.position.y;
        mesh.position.z = pose.position.z;
        return mesh
    }
}
