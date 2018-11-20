import * as THREE from "three";

/*
import 'three/Line2';
import 'three/LineMaterial';
import 'three/LineGeometry';
*/


import {steps} from "../../../../../model/Redux/Page/RouteCodeEditor";


export default class Waypoint extends THREE.Group {

  constructor() {
    super();

    this.waypoint = null;
    this.lane = null;

    this.waypointsList = {};
    this.laneList = {};
    this.arrowHelper = {};

    this.intersected = null;

    this.camera = null;
    this.controls = null;

    this.activeStep = null;

    this.isBack = false;
    this.startPoint = null;
    this.lanes = [];
    this.endPoint = null;

    this.setStartPoint = null;
    this.setLaneList = null;
    this.setEndPoint = null;
    this.clearRouteCodeData = null;

    this.nextLanes = [];
    this.endPointCandidate = [];

    this.selectCandidateObject = [];

    this.raycaster = new THREE.Raycaster();


    this.color = {
      default: "#d5cdce",
      selectCandidate: "#ffff0b",
      selected: "#00FF00"
    }

  }

  set3DParameter(camera, controls) {
    this.camera = camera;
    this.controls = controls;
  }

  setWaypoint(waypoint, lane) {

    this.waypoint = waypoint;
    this.lane = lane;

    let waypoints = waypoint.waypoints;
    let lanes = lane.lanes;

    if (Object.keys(lanes).length > 0 && Object.keys(waypoints).length > 0) {
      const color = new THREE.Color(this.color.default);

      for (const waypointID in waypoints) {
        if (waypoints.hasOwnProperty(waypointID)) {
          const sphere = new THREE.SphereBufferGeometry(0.1, 16, 8);
          const waypoint = new THREE.Mesh(sphere, new THREE.MeshBasicMaterial({
            color: color
          }));
          waypoint.position.set(waypoints[waypointID].x, waypoints[waypointID].y, waypoints[waypointID].z);
          waypoint.userData = {
            waypointID: waypointID
          };
          waypoint.name = waypointID;
          this.waypointsList[waypointID] = waypoint;
          this.add(this.waypointsList[waypointID]);
        }
      }

      for (const laneID in lanes) {
        if (lanes.hasOwnProperty(laneID)) {

          let material = new THREE.LineBasicMaterial({color: color, linewidth: 5});
          let geometry = new THREE.Geometry();
          const waypointIDs = lanes[laneID].waypointIDs;
          for (const waypointID of waypointIDs) {
            geometry.vertices.push(new THREE.Vector3(
              waypoints[waypointID].x,
              waypoints[waypointID].y,
              waypoints[waypointID].z));
            this.waypointsList[waypointID].userData.laneCode = laneID;
          }
          geometry.computeBoundingSphere();
          const line = new THREE.Line(geometry, material);


          /*
          let matLine = new THREE.LineMaterial({
            color: 0xffffff,
            linewidth: 5,
            vertexColors: THREE.VertexColors,
            dashed: false
          });
          let geometry = new THREE.LineGeometry();

          let positions = [];
          for (const waypointID of waypointIDs) {
            positions.push(
              waypoints[waypointID].x,
              waypoints[waypointID].y,
              waypoints[waypointID].z);
            this.waypointsList[waypointID].userData.laneCode = laneID;
          }
          geometry.setPosition(positions);
          const line = new THREE.Line2(geometry, matLine);
          line.computeLineDistances();
          line.scale.set( 1, 1, 1 );
          */

          line.name = laneID;
          this.laneList[laneID] = line;
          this.add(this.laneList[laneID]);
          let to = geometry.vertices[geometry.vertices.length - 1];
          let from = geometry.vertices[geometry.vertices.length - 2];
          let direction = to.clone().sub(from);
          const headLength = 0.5;
          const headWidth = 0.25;
          let arrowHelper = new THREE.ArrowHelper(direction.normalize(), to, headLength + 0.00000001, color, headLength, headWidth);
          arrowHelper.name = "laneHead/" + laneID;
          this.arrowHelper[laneID] = arrowHelper;
          this.add(this.arrowHelper[laneID]);
        }
      }

      let newCameraPosition = {
        x: waypoints[Object.keys(waypoints)[0]].x,
        y: waypoints[Object.keys(waypoints)[0]].y,
        z: waypoints[Object.keys(waypoints)[0]].z
      };
      this.updateCameraPosition(newCameraPosition);
    }
  }

  clear() {
    for (const meshName in this.waypointsList) {
      if (this.waypointsList.hasOwnProperty(meshName)) {
        const threeJSObject = this.getObjectByName(meshName);
        if (typeof threeJSObject !== "undefined") {
          this.remove(threeJSObject);
        }
        delete this.waypointsList[meshName];
      }
    }

    for (const meshName in this.laneList) {
      if (this.laneList.hasOwnProperty(meshName)) {
        const threeJSObject = this.getObjectByName(meshName);
        if (typeof threeJSObject !== "undefined") {
          this.remove(threeJSObject);
        }
        delete this.laneList[meshName];
      }
    }

    for (const meshName in this.arrowHelper) {
      if (this.arrowHelper.hasOwnProperty(meshName)) {
        const threeJSObject = this.getObjectByName("laneHead/" + meshName);
        if (typeof threeJSObject !== "undefined") {
          this.remove(threeJSObject);
        }
        delete this.arrowHelper[meshName];
      }
    }

    this.waypointsList = {};
    this.laneList = {};
    this.arrowHelper = {};
  }

  changeObjectColorToDefault() {
    if (this.startPoint !== null) {
      this.waypointsList[this.startPoint].material.color.set(this.color.default);
    }
    for (const laneID of this.lanes) {
      this.laneList[laneID].material.color.set(this.color.default);
    }
    for (let laneID of this.nextLanes) {
      this.laneList[laneID].material.color.set(this.color.default);
    }

    if (this.endPoint !== null) {
      this.waypointsList[this.endPoint].material.color.set(this.color.default);
    }
    for (let waypointID of this.endPointCandidate) {
      this.waypointsList[waypointID].material.color.set(this.color.default);
    }
  }

  setActiveStep(activeStep) {
    this.activeStep = activeStep;
    this.selectCandidateObject = [];

    if (activeStep === steps.selectStartPoint.id) {
      this.changeObjectColorToDefault();
      if (this.startPoint !== null) {
        this.intersected = this.waypointsList[this.startPoint];
        this.waypointsList[this.startPoint].material.color.set(this.color.selected);
      }
      this.selectCandidateObject = Object.values(this.waypointsList);
    } else if (activeStep === steps.selectLane.id) {
      this.changeObjectColorToDefault();
      if (this.startPoint !== null) {
        this.waypointsList[this.startPoint].material.color.set(this.color.selected);
      }
      for (let laneID of this.lanes) {
        this.intersected = this.laneList[laneID];
        this.laneList[laneID].material.color.set(this.color.selected);
      }
      for (let laneID of this.nextLanes) {
        this.laneList[laneID].material.color.set(this.color.selectCandidate);
        this.selectCandidateObject.push(this.laneList[laneID]);
      }
    } else if (activeStep === steps.selectEndPoint.id) {
      this.changeObjectColorToDefault();
      this.intersected = null;
      this.endPoint = null;
      if (this.startPoint !== null) {
        this.waypointsList[this.startPoint].material.color.set(this.color.selected);
      }
      for (let waypointID of this.endPointCandidate) {
        this.waypointsList[waypointID].material.color.set(this.color.selectCandidate);
        this.selectCandidateObject.push(this.waypointsList[waypointID]);
      }
      for (let laneID of this.lanes) {
        this.laneList[laneID].material.color.set(this.color.selected);
      }
    } else if (activeStep === steps.advanceOrBack.id) {
      this.changeObjectColorToDefault();

      this.startPoint = null;
      this.lanes = [];
      this.endPoint = null;
      this.nextLanes = [];
      this.endPointCandidate = [];
      this.selectCandidateObject = [];

    }
  }

  setIsBack(isBack) {
    this.isBack = isBack;
  }


  /*
  setRouteCodeData(startPoint, laneList, endPoint){
    if (this.activeStep === steps.selectStartPoint.id) {
      this.reflectStartPointStep(startPoint, laneList, endPoint);
    } else if (this.activeStep === steps.selectLane.id) {
      this.reflectLaneListStep(startPoint, laneList, endPoint);
    } else if (this.activeStep === steps.selectEndPoint.id) {
      this.reflectEndPointStep(startPoint, laneList, endPoint);
    }
  }

  reflectStartPointStep(startPoint, laneList, endPoint){
    this.startPoint = startPoint;
    this.lanes = laneList;
    this.endPointCandidate = this.lane.lanes[this.waypointsList[startPoint].userData.laneCode].waypointIDs;
    this.getNextLanes(this.waypointsList[startPoint].userData.laneCode);
    this.waypointsList[startPoint].material.color.set(this.color.selected);
  }

  reflectLaneListStep(startPoint, laneList, endPoint){

  }

  reflectEndPointStep(startPoint, laneList, endPoint){

  }
  */


  setCallback(setStartPoint, setLaneList, setEndPoint, clearRouteCodeData) {
    this.setStartPoint = setStartPoint;
    this.setLaneList = setLaneList;
    this.setEndPoint = setEndPoint;
    this.clearRouteCodeData = clearRouteCodeData;
  }

  selectObject(mouse) {
    if (this.activeStep === steps.selectStartPoint.id) {
      this.selectStartPoint(mouse);
    } else if (this.activeStep === steps.selectLane.id) {
      this.selectLane(mouse);
    } else if (this.activeStep === steps.selectEndPoint.id) {
      this.selectEndPoint(mouse);
    }
  }

  selectStartPoint(mouse) {
    this.raycaster.setFromCamera(mouse, this.camera);
    let intersects = this.raycaster.intersectObjects(this.selectCandidateObject);
    if (intersects.length > 0) {
      if (this.intersected !== intersects[0].object) {
        if (this.intersected) this.intersected.material.color.set(this.color.default);
        this.intersected = intersects[0].object;

        this.startPoint = this.intersected.userData.waypointID;
        this.lanes = [this.intersected.userData.laneCode];
        this.endPointCandidate = this.lane.lanes[this.intersected.userData.laneCode].waypointIDs;

        this.getNextLanes(this.intersected.userData.laneCode);

        this.intersected.material.color.set(this.color.selected);
      }
    } else {
      if (this.intersected) this.intersected.material.color.set(this.color.default);
      this.intersected = null;
    }

    this.setStartPoint(this.startPoint);
    this.setLaneList(this.lanes);

  }

  selectLane(mouse) {
    this.raycaster.setFromCamera(mouse, this.camera);
    let intersects = this.raycaster.intersectObjects(this.selectCandidateObject);

    if (intersects.length > 0) {

      if (this.intersected !== intersects[0].object) {
        this.intersected = intersects[0].object;
        for (let laneID of this.nextLanes) {
          this.laneList[laneID].material.color.set(this.color.default);
        }

        this.lanes.push(this.intersected.name);
        this.getNextLanes(this.intersected.name);
        this.endPointCandidate = this.lane.lanes[this.intersected.name].waypointIDs;

        this.intersected.material.color.set(this.color.selected);

        this.selectCandidateObject = [];
        for (let laneID of this.nextLanes) {
          this.laneList[laneID].material.color.set(this.color.selectCandidate);
          this.selectCandidateObject.push(this.laneList[laneID]);
        }
        this.selectCandidateObject.push(this.intersected);

      } else {

        this.intersected.material.color.set(this.color.default);
        for (let laneID of this.nextLanes) {
          this.laneList[laneID].material.color.set(this.color.default);
        }

        this.lanes.pop();
        this.selectCandidateObject = [];

        this.intersected = this.laneList[this.lanes[this.lanes.length - 1]];
        this.intersected.material.color.set(this.color.selected);
        this.getNextLanes(this.lanes[this.lanes.length - 1]);

        for (let laneID of this.nextLanes) {
          this.laneList[laneID].material.color.set(this.color.selectCandidate);
          this.selectCandidateObject.push(this.laneList[laneID]);
        }
        this.selectCandidateObject.push(this.intersected);

      }
    }

    this.setLaneList(this.lanes);
    console.log(this.startPoint, this.lanes);
  }

  selectEndPoint(mouse) {
    this.raycaster.setFromCamera(mouse, this.camera);
    let intersects = this.raycaster.intersectObjects(this.selectCandidateObject);
    if (intersects.length > 0) {
      if (this.intersected !== intersects[0].object) {
        if (this.intersected) this.intersected.material.color.set(this.color.selectCandidate);
        this.intersected = intersects[0].object;

        this.endPoint = this.intersected.userData.waypointID;

        this.intersected.material.color.set(this.color.selected);
      }
    } else {
      if (this.intersected) this.intersected.material.color.set(this.color.selectCandidate);
      this.intersected = null;
      this.endPoint = null;
    }

    this.setEndPoint(this.endPoint);
    console.log(this.endPoint);

  }

  getNextLanes(laneID) {
    let nextLanes = this.isBack ? this.lane.fromLanes[laneID] : this.lane.toLanes[laneID];
    this.nextLanes = nextLanes ? nextLanes : [];
  }

  updateCameraPosition(newCameraPosition) {
    this.camera.position.x = newCameraPosition.x;
    this.camera.position.y = newCameraPosition.y;
    this.camera.position.z = newCameraPosition.z + 200;
    this.controls.target.set(
      newCameraPosition.x,
      newCameraPosition.y,
      newCameraPosition.z);
  }

}
