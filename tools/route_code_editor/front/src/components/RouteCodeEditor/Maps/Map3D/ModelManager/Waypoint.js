import * as THREE from "three";

import '../../../../../lib/lines/LineSegments2.js';
import '../../../../../lib/lines/Line2';
import '../../../../../lib/lines/LineMaterial';
import '../../../../../lib/lines/LineSegmentsGeometry';
import '../../../../../lib/lines/LineGeometry';

import {steps} from "../../../../../model/Redux/Page/RouteCodeEditor";


export default class Waypoint extends THREE.Group {

  constructor() {
    super();

    this.waypoint = null;
    this.lane = null;

    this.waypointsList = {};
    this.laneList = {};
    this.arrowHelper = {};

    this.camera = null;
    this.controls = null;

    this.activeStep = null;

    this.isBack = false;
    this.startPoint = null;
    this.lanes = [];
    this.endPoint = null;

    this.setStartPointAndLaneList = null;
    this.setLaneList = null;
    this.setEndPoint = null;

    this.selectCandidateObject = [];

    this.raycaster = new THREE.Raycaster();


    this.color = {
      default: "#d5cdce",
      selectCandidate: "#ff0016",
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
          const sphere = new THREE.SphereBufferGeometry(0.2, 16, 8);
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

          /*
          let matLine = new THREE.LineMaterial({
            color: 0xffffff,
            linewidth: 5,
            vertexColors: THREE.VertexColors,
            dashed: false
          });
          let geometry = new THREE.LineGeometry();
          const waypointIDs = lanes[laneID].waypointIDs;

          let positions = [];
          let colors = [];
          let lineColor = new THREE.Color();

          for (const waypointID of waypointIDs) {
            positions.push(
              waypoints[waypointID].x,
              waypoints[waypointID].y,
              waypoints[waypointID].z);
            lineColor.setHSL( waypointID / Math.max(waypointIDs), 1.0, 0.5 );
            colors.push( lineColor.r, lineColor.g, lineColor.b );
            this.waypointsList[waypointID].userData.laneCode = laneID;
          }
          geometry.setPositions(positions);
          //geometry.setColors( colors );
          const line = new THREE.Line2(geometry, matLine);
          line.computeLineDistances();
          line.scale.set( 1, 1, 1 );

          line.name = laneID;
          this.laneList[laneID] = line;
          this.add(this.laneList[laneID]);
          */


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

    this.waypoint = null;
    this.lane = null;

    this.waypointsList = {};
    this.laneList = {};
    this.arrowHelper = {};
  }

  setIsBack(isBack) {
    this.isBack = isBack;
  }



  changeObjectColorToDefault() {

    if (this.startPoint !== null) {
      this.waypointsList[this.startPoint].material.color.set(this.color.default);
    }
    for (const laneID of this.lanes) {
      this.laneList[laneID].material.color.set(this.color.default);
    }
    if(this.lanes.length > 0) {
      for (let laneID of this.getNextLanes(this.lanes[this.lanes.length - 1])) {
        this.laneList[laneID].material.color.set(this.color.default);
      }
    }
    if (this.endPoint !== null) {
      this.waypointsList[this.endPoint].material.color.set(this.color.default);
    }
    if(this.lanes.length > 0) {
      for (let waypointID of this.lane.lanes[this.lanes[this.lanes.length - 1]].waypointIDs) {
        this.waypointsList[waypointID].material.color.set(this.color.default);
      }
    }
  }

  setActiveStep(activeStep) {

    if(this.waypoint !==null && this.lane !== null) {

      this.activeStep = activeStep;
      this.selectCandidateObject = [];

      if (activeStep === steps.selectStartPoint.id) {
        this.changeObjectColorToDefault();
        if (this.startPoint !== null) {
          this.waypointsList[this.startPoint].material.color.set(this.color.selected);
        }
        this.selectCandidateObject = Object.values(this.waypointsList);
      } else if (activeStep === steps.selectLane.id) {
        this.changeObjectColorToDefault();
        if (this.startPoint !== null) {
          this.waypointsList[this.startPoint].material.color.set(this.color.selected);
        }
        for (let laneID of this.lanes) {
          this.laneList[laneID].material.color.set(this.color.selected);
        }
        for (let laneID of this.getNextLanes(this.lanes[this.lanes.length - 1])) {
          this.laneList[laneID].material.color.set(this.color.selectCandidate);
          this.selectCandidateObject.push(this.laneList[laneID]);
        }
        this.selectCandidateObject.push(this.laneList[this.lanes[this.lanes.length - 1]]);
      } else if (activeStep === steps.selectEndPoint.id) {
        this.changeObjectColorToDefault();
        if (this.startPoint !== null) {
          this.waypointsList[this.startPoint].material.color.set(this.color.selected);
        }
        for (let waypointID of this.lane.lanes[this.lanes[this.lanes.length - 1]].waypointIDs) {
          this.waypointsList[waypointID].material.color.set(this.color.selectCandidate);
          this.selectCandidateObject.push(this.waypointsList[waypointID]);
        }
        for (let laneID of this.lanes) {
          this.laneList[laneID].material.color.set(this.color.selected);
        }
      } else if (activeStep === steps.result.id) {
        this.changeObjectColorToDefault();
        if (this.startPoint !== null) {
          this.waypointsList[this.startPoint].material.color.set(this.color.selected);
        }
        for (let laneID of this.lanes) {
          this.laneList[laneID].material.color.set(this.color.selected);
        }
        if (this.endPoint !== null) {
          this.waypointsList[this.endPoint].material.color.set(this.color.selected);
        }
      } else if (activeStep === steps.advanceOrBack.id) {
        this.changeObjectColorToDefault();
        this.startPoint = null;
        this.lanes = [];
        this.endPoint = null;
        this.selectCandidateObject = [];
      }
    }
  }


  updateRouteCode(startPoint, lanes, endPoint) {

    console.log(startPoint, lanes, endPoint);

    if (this.waypoint !== null && this.lane !== null) {

      this.changeObjectColorToDefault();
      this.selectCandidateObject = [];
      this.startPoint = startPoint !== "" ? startPoint : null;
      this.lanes = lanes;
      this.endPoint = endPoint !== "" ? endPoint : null;

      if (this.activeStep === steps.selectStartPoint.id) {
        if (this.startPoint !== null) {
          this.waypointsList[this.startPoint].material.color.set(this.color.selected);
        }
        this.selectCandidateObject = Object.values(this.waypointsList);
      } else if (this.activeStep === steps.selectLane.id) {
        if (this.startPoint !== null) {
          this.waypointsList[this.startPoint].material.color.set(this.color.selected);
        }
        for (let laneID of this.lanes) {
          this.laneList[laneID].material.color.set(this.color.selected);
        }
        for (let laneID of this.getNextLanes(this.lanes[this.lanes.length - 1])) {
          this.laneList[laneID].material.color.set(this.color.selectCandidate);
          this.selectCandidateObject.push(this.laneList[laneID]);
        }
        this.selectCandidateObject.push(this.laneList[this.lanes[this.lanes.length - 1]]);

      } else if (this.activeStep === steps.selectEndPoint.id) {
        if (this.startPoint !== null) {
          this.waypointsList[this.startPoint].material.color.set(this.color.selected);
        }
        for (let waypointID of this.lane.lanes[this.lanes[this.lanes.length - 1]].waypointIDs) {
          this.waypointsList[waypointID].material.color.set(this.color.selectCandidate);
          this.selectCandidateObject.push(this.waypointsList[waypointID]);
        }
        for (let laneID of this.lanes) {
          this.laneList[laneID].material.color.set(this.color.selected);
        }
        if (this.endPoint !== null) {
          this.waypointsList[this.endPoint].material.color.set(this.color.selected);
        }
      } else if (this.activeStep === steps.advanceOrBack.id) {
        this.startPoint = null;
        this.lanes = [];
        this.endPoint = null;
        this.selectCandidateObject = [];
      }
    }
  }


  setCallback(setStartPointAndLaneList, setLaneList, setEndPoint) {
    this.setStartPointAndLaneList = setStartPointAndLaneList;
    this.setLaneList = setLaneList;
    this.setEndPoint = setEndPoint;
  }

  selectObject(mouse) {
    if(this.waypoint !==null && this.lane !== null) {
      if (this.activeStep === steps.selectStartPoint.id) {
        this.selectStartPoint(mouse);
      } else if (this.activeStep === steps.selectLane.id) {
        this.selectLane(mouse);
      } else if (this.activeStep === steps.selectEndPoint.id) {
        this.selectEndPoint(mouse);
      }
    }
  }

  selectStartPoint(mouse) {
    this.raycaster.setFromCamera(mouse, this.camera);
    let intersects = this.raycaster.intersectObjects(this.selectCandidateObject);
    let startPoint = "";
    let lanes = [];

    if (intersects.length > 0) {
      if (this.startPoint !== intersects[0].object.name) {
        startPoint = intersects[0].object.userData.waypointID;
        lanes = [intersects[0].object.userData.laneCode];
      }
    }

    this.setStartPointAndLaneList(startPoint, lanes);

  }

  selectLane(mouse) {
    this.raycaster.setFromCamera(mouse, this.camera);
    let intersects = this.raycaster.intersectObjects(this.selectCandidateObject);

    let lanes = this.lanes.slice(0, this.lanes.length);

    if (intersects.length > 0) {

      if (lanes[lanes.length - 1] !== intersects[0].object.name) {
        lanes.push(intersects[0].object.name);
      } else {
        if(lanes.length > 1) {
          lanes.pop();
        }
      }
    }

    this.setLaneList(lanes);
    console.log(lanes);
  }

  selectEndPoint(mouse) {
    this.raycaster.setFromCamera(mouse, this.camera);
    let intersects = this.raycaster.intersectObjects(this.selectCandidateObject);
    let endPoint = "";

    if (intersects.length > 0) {
      if (this.endPoint !== intersects[0].object.name) {
        endPoint = intersects[0].object.userData.waypointID;
      }
    }

    this.setEndPoint(endPoint);
    console.log(endPoint);

  }

  getNextLanes(laneID) {
    if(this.lane !== null) {
      let nextLanes = this.isBack ? this.lane.fromLanes[laneID] : this.lane.toLanes[laneID];
      return nextLanes ? nextLanes : [];
    }else{
      return []
    }
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
