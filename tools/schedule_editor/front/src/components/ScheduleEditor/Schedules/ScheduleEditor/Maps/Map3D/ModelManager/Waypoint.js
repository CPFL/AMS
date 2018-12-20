import * as THREE from 'three';

import { steps } from '../../../../../../../model/Redux/Page/ScheduleEditor';

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
      default: '#d5cdce',
      selectCandidate: '#ff0016',
      selected: '#00FF00'
    };
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
          const waypoint = new THREE.Mesh(
            sphere,
            new THREE.MeshBasicMaterial({
              color: color
            })
          );
          waypoint.position.set(
            waypoints[waypointID].x,
            waypoints[waypointID].y,
            waypoints[waypointID].z
          );
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
          let positions = [];
          const waypointIDs = lanes[laneID].waypointIDs;

          for (const waypointID of waypointIDs) {
            positions.push(
              new THREE.Vector3(
                waypoints[waypointID].x,
                waypoints[waypointID].y,
                waypoints[waypointID].z
              )
            );
            this.waypointsList[waypointID].userData.laneCode = laneID;
          }

          let path = new THREE.CatmullRomCurve3(positions);
          let geometry = new THREE.TubeBufferGeometry(path, 64, 0.1, 8, false);
          let material = new THREE.MeshBasicMaterial({
            color: this.color.default
          });
          let line = new THREE.Mesh(geometry, material);
          line.name = laneID;
          this.laneList[laneID] = line;

          this.add(this.laneList[laneID]);

          let to = new THREE.Vector3(
            waypoints[waypointIDs[waypointIDs.length - 1]].x,
            waypoints[waypointIDs[waypointIDs.length - 1]].y,
            waypoints[waypointIDs[waypointIDs.length - 1]].z
          );
          let from = new THREE.Vector3(
            waypoints[waypointIDs[waypointIDs.length - 2]].x,
            waypoints[waypointIDs[waypointIDs.length - 2]].y,
            waypoints[waypointIDs[waypointIDs.length - 2]].z
          );
          let direction = to.clone().sub(from);
          const headLength = 0.8;
          const headWidth = 0.8;
          let arrowHelper = new THREE.ArrowHelper(
            direction.normalize(),
            to,
            headLength + 0.00000001,
            color,
            headLength,
            headWidth
          );
          arrowHelper.name = 'laneHead/' + laneID;
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
        if (typeof threeJSObject !== 'undefined') {
          this.remove(threeJSObject);
        }
        delete this.waypointsList[meshName];
      }
    }

    for (const meshName in this.laneList) {
      if (this.laneList.hasOwnProperty(meshName)) {
        const threeJSObject = this.getObjectByName(meshName);
        if (typeof threeJSObject !== 'undefined') {
          this.remove(threeJSObject);
        }
        delete this.laneList[meshName];
      }
    }

    for (const meshName in this.arrowHelper) {
      if (this.arrowHelper.hasOwnProperty(meshName)) {
        const threeJSObject = this.getObjectByName('laneHead/' + meshName);
        if (typeof threeJSObject !== 'undefined') {
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

  changeRouteCodeColorToDefault() {
    if (this.startPoint !== null) {
      this.waypointsList[this.startPoint].material.color.set(
        this.color.default
      );
    }
    for (const laneID of this.lanes) {
      this.laneList[laneID].material.color.set(this.color.default);
    }
    if (this.endPoint !== null) {
      this.waypointsList[this.endPoint].material.color.set(this.color.default);
    }
  }

  setActiveStep(activeStep) {
    this.activeStep = activeStep;
    if (this.waypoint !== null && this.lane !== null) {
      this.selectCandidateObject = [];

      if (activeStep === steps.advanceOrBack.id) {
        this.changeObjectColorToDefault();
        this.startPoint = null;
        this.lanes = [];
        this.endPoint = null;
        this.selectCandidateObject = [];
      } else if (activeStep === steps.selectStartPoint.id) {
        this.changeObjectColorToDefault();
        if (this.startPoint !== null) {
          this.waypointsList[this.startPoint].material.color.set(
            this.color.selected
          );
        }
        this.selectCandidateObject = Object.values(this.waypointsList);
      } else if (activeStep === steps.selectLane.id) {
        this.changeObjectColorToDefault();
        if (this.startPoint !== null) {
          this.waypointsList[this.startPoint].material.color.set(
            this.color.selected
          );
        }
        for (let laneID of this.lanes) {
          this.laneList[laneID].material.color.set(this.color.selected);
        }
        for (let laneID of this.getNextLanes(
          this.lanes[this.lanes.length - 1]
        )) {
          this.laneList[laneID].material.color.set(this.color.selectCandidate);
          this.selectCandidateObject.push(this.laneList[laneID]);
        }
        this.selectCandidateObject.push(
          this.laneList[this.lanes[this.lanes.length - 1]]
        );
      } else if (activeStep === steps.selectEndPoint.id) {
        this.changeObjectColorToDefault();
        if (this.startPoint !== null) {
          this.waypointsList[this.startPoint].material.color.set(
            this.color.selected
          );
        }
        if (this.lanes.length === 1) {
          const index = this.lane.lanes[
            this.lanes[this.lanes.length - 1]
          ].waypointIDs.indexOf(this.startPoint);
          let selectCandidateWaypointIDs = !this.isBack
            ? this.lane.lanes[
                this.lanes[this.lanes.length - 1]
              ].waypointIDs.slice(index + 1)
            : this.lane.lanes[
                this.lanes[this.lanes.length - 1]
              ].waypointIDs.slice(0, index - 1);
          for (const waypointID of selectCandidateWaypointIDs) {
            this.waypointsList[waypointID].material.color.set(
              this.color.selectCandidate
            );
            this.selectCandidateObject.push(this.waypointsList[waypointID]);
          }
        } else {
          for (let waypointID of this.lane.lanes[
            this.lanes[this.lanes.length - 1]
          ].waypointIDs) {
            this.waypointsList[waypointID].material.color.set(
              this.color.selectCandidate
            );
            this.selectCandidateObject.push(this.waypointsList[waypointID]);
          }
        }
        for (let laneID of this.lanes) {
          this.laneList[laneID].material.color.set(this.color.selected);
        }
      } else if (activeStep === steps.result.id) {
        this.changeObjectColorToDefault();
        if (this.startPoint !== null) {
          this.waypointsList[this.startPoint].material.color.set(
            this.color.selected
          );
        }
        for (let laneID of this.lanes) {
          this.laneList[laneID].material.color.set(this.color.selected);
        }
        if (this.endPoint !== null) {
          this.waypointsList[this.endPoint].material.color.set(
            this.color.selected
          );
        }
      }
    }
  }

  updateRouteCode(routeCode) {

    console.log(routeCode);
    if (this.waypoint !== null && this.lane !== null && routeCode) {
      const startPoint = routeCode.startPoint;
      const lanes = routeCode.laneList;
      const endPoint = routeCode.endPoint;

      this.changeRouteCodeColorToDefault();
      this.selectCandidateObject = [];
      this.startPoint = startPoint !== '' ? startPoint : null;
      this.lanes = lanes;
      this.endPoint = endPoint !== '' ? endPoint : null;

      console.log(routeCode);

      if (this.startPoint) {
        this.waypointsList[this.startPoint].material.color.set(
          this.color.selected
        );
      }
      for (let laneID of this.lanes) {
        this.laneList[laneID].material.color.set(this.color.selected);
      }
      if (this.endPoint) {
        this.waypointsList[this.endPoint].material.color.set(
          this.color.selected
        );
      }
    }
  }

  setCallback(setStartPointAndLaneList, setLaneList, setEndPoint) {
    this.setStartPointAndLaneList = setStartPointAndLaneList;
    this.setLaneList = setLaneList;
    this.setEndPoint = setEndPoint;
  }

  selectObject(mouse) {
    if (this.waypoint !== null && this.lane !== null) {
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
    let intersects = this.raycaster.intersectObjects(
      this.selectCandidateObject
    );
    let startPoint = '';
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
    let intersects = this.raycaster.intersectObjects(
      this.selectCandidateObject
    );

    let lanes = this.lanes.slice(0, this.lanes.length);

    if (intersects.length > 0) {
      if (lanes[lanes.length - 1] !== intersects[0].object.name) {
        lanes.push(intersects[0].object.name);
      } else {
        if (lanes.length > 1) {
          lanes.pop();
        }
      }
    }
    this.setLaneList(lanes);
  }

  selectEndPoint(mouse) {
    this.raycaster.setFromCamera(mouse, this.camera);
    let intersects = this.raycaster.intersectObjects(
      this.selectCandidateObject
    );
    let endPoint = '';

    if (intersects.length > 0) {
      if (this.endPoint !== intersects[0].object.name) {
        endPoint = intersects[0].object.userData.waypointID;
      }
    }

    this.setEndPoint(endPoint);
  }

  getNextLanes(laneID) {
    if (this.lane !== null) {
      let nextLanes = this.isBack
        ? this.lane.fromLanes[laneID]
        : this.lane.toLanes[laneID];
      return nextLanes ? nextLanes : [];
    } else {
      return [];
    }
  }

  updateChangeRoute(startPoint, lanes, endPoint, decisionSectionEndPoint) {
    console.log(startPoint, lanes, endPoint, decisionSectionEndPoint);
  }

  focusCamera() {
    switch (this.activeStep) {
      case steps.advanceOrBack.id:
        break;
      case steps.selectStartPoint.id:
        if (this.startPoint !== null) {
          const position = this.waypointsList[this.startPoint].position;
          let newCameraPosition = {
            x: position.x,
            y: position.y,
            z: position.z
          };
          this.updateCameraPosition(newCameraPosition);
        }
        break;
      case steps.selectLane.id:
        if (this.lanes.length) {
          const position = this.laneList[this.lanes[this.lanes.length - 1]]
            .geometry.parameters.path.points[0];
          let newCameraPosition = {
            x: position.x,
            y: position.y,
            z: position.z
          };
          this.updateCameraPosition(newCameraPosition);
        } else if (this.startPoint !== null) {
          const position = this.waypointsList[this.startPoint].position;
          let newCameraPosition = {
            x: position.x,
            y: position.y,
            z: position.z
          };
          this.updateCameraPosition(newCameraPosition);
        }
        break;
      case steps.selectEndPoint.id:
        if (this.endPoint !== null) {
          const position = this.waypointsList[this.endPoint].position;
          let newCameraPosition = {
            x: position.x,
            y: position.y,
            z: position.z
          };
          this.updateCameraPosition(newCameraPosition);
        } else if (this.lanes.length) {
          const position = this.laneList[this.lanes[this.lanes.length - 1]]
            .geometry.parameters.path.points[0];
          let newCameraPosition = {
            x: position.x,
            y: position.y,
            z: position.z
          };
          this.updateCameraPosition(newCameraPosition);
        }
        break;
      case steps.result.id:
        if (this.startPoint !== null) {
          const position = this.waypointsList[this.startPoint].position;
          let newCameraPosition = {
            x: position.x,
            y: position.y,
            z: position.z
          };
          this.updateCameraPosition(newCameraPosition);
        }
        break;
      default:
    }
  }

  updateCameraPosition(newCameraPosition) {
    this.camera.position.x = newCameraPosition.x;
    this.camera.position.y = newCameraPosition.y;
    this.camera.position.z = newCameraPosition.z + 100;
    this.controls.target.set(
      newCameraPosition.x,
      newCameraPosition.y,
      newCameraPosition.z
    );
  }
}
