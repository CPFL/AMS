import * as THREE from 'three';

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

    this.selectRouteCode = null;
    this.scheduleList = null;
    this.routeCodeList = null;
    this.nextSelectableRouteList = null;

    this.setStartPointAndLaneList = null;
    this.setLaneList = null;
    this.setEndPoint = null;

    this.selectCandidateObject = [];

    this.raycaster = new THREE.Raycaster();

    this.color = {
      default: '#d5cdce',
      selectCandidate: '#ff0016',
      selected: '#00FF00',
      decided: '#1121ff'
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

  changeScheduleListColorToDefault() {
    if (this.scheduleList) {
      for (const schedule of this.scheduleList) {
        if (schedule.startPoint) {
          this.waypointsList[schedule.startPoint].material.color.set(
            this.color.default
          );
        }
        for (const laneID of schedule.laneList) {
          this.laneList[laneID].material.color.set(this.color.default);
        }
        if (schedule.endPoint) {
          this.waypointsList[schedule.endPoint].material.color.set(
            this.color.default
          );
        }
      }
    }
  }

  initScheduleList(scheduleList) {
    if (scheduleList && this.waypoint && this.lane) {
      this.changeScheduleListColorToDefault();
      this.scheduleList = scheduleList;

      for (const schedule of scheduleList) {
        const startPoint = schedule.startPoint;
        const lanes = schedule.laneList;
        const endPoint = schedule.endPoint;

        if (startPoint && lanes.length > 0 && endPoint) {
          this.waypointsList[startPoint].material.color.set(this.color.decided);
          for (let laneID of lanes) {
            this.laneList[laneID].material.color.set(this.color.decided);
          }
          this.waypointsList[endPoint].material.color.set(this.color.decided);

          const newCameraPosition = {
            x: this.waypoint.waypoints[endPoint].x,
            y: this.waypoint.waypoints[endPoint].y,
            z: this.waypoint.waypoints[endPoint].z
          };
          this.updateCameraPosition(newCameraPosition);
        }
      }
    }
  }

  initRouteCodeList(routeCodeList) {
    this.routeCodeList = routeCodeList;
  }

  changeCurrentRouteCodeColorToDefault() {
    if (this.selectRouteCode) {
      if (this.selectRouteCode.startPoint) {
        this.waypointsList[this.selectRouteCode.startPoint].material.color.set(
          this.color.default
        );
      }
      for (const laneID of this.selectRouteCode.laneList) {
        this.laneList[laneID].material.color.set(this.color.default);
      }
      if (this.selectRouteCode.endPoint) {
        this.waypointsList[this.selectRouteCode.endPoint].material.color.set(
          this.color.default
        );
      }
    }
    if (this.nextSelectableRouteList) {
      for (const routeCode of this.nextSelectableRouteList) {
        if (routeCode.startPoint) {
          this.waypointsList[routeCode.startPoint].material.color.set(
            this.color.default
          );
        }
        for (const laneID of routeCode.laneList) {
          this.laneList[laneID].material.color.set(this.color.default);
        }
        if (routeCode.endPoint) {
          this.waypointsList[routeCode.endPoint].material.color.set(
            this.color.default
          );
        }
      }
    }
  }

  updateCurrentRouteCode(selectRouteCode) {
    const startPoint = selectRouteCode.startPoint;
    const lanes = selectRouteCode.laneList;
    const endPoint = selectRouteCode.endPoint;
    if (
      this.waypoint &&
      this.lane &&
      startPoint &&
      lanes.length > 0 &&
      endPoint
    ) {
      this.changeCurrentRouteCodeColorToDefault();

      this.selectRouteCode = selectRouteCode;

      this.waypointsList[startPoint].material.color.set(this.color.selected);
      for (let laneID of lanes) {
        this.laneList[laneID].material.color.set(this.color.selected);
      }
      this.waypointsList[endPoint].material.color.set(this.color.selected);

      const newCameraPosition = {
        x: this.waypoint.waypoints[endPoint].x,
        y: this.waypoint.waypoints[endPoint].y,
        z: this.waypoint.waypoints[endPoint].z
      };
      this.updateCameraPosition(newCameraPosition);

      this.nextSelectableRouteList = [];
      for (const routeCode of this.routeCodeList) {
        const routeStartPoint = routeCode.startPoint;
        if (endPoint === routeStartPoint) {
          /*
          this.waypointsList[routeCode.startPoint].material.color.set(
            this.color.selectCandidate
          );
          for (let laneID of routeCode.laneList) {
            this.laneList[laneID].material.color.set(
              this.color.selectCandidate
            );
          }
          */
          routeCode.laneList.forEach((laneID, index) => {
            if (index > 0) {
              this.laneList[laneID].material.color.set(
                this.color.selectCandidate
              );
            }
          });
          this.waypointsList[routeCode.endPoint].material.color.set(
            this.color.selectCandidate
          );
          this.nextSelectableRouteList.push(routeCode);
        }
      }
    }
  }

  updateChangeRoute(startPoint, lanes, endPoint, decisionSectionEndPoint) {
    console.log(startPoint, lanes, endPoint, decisionSectionEndPoint);
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
