import { Waypoint, WaypointData } from './Waypoint';
export default class Waypoints extends THREE.Group {

  constructor(name) {
    super();
    this.name = name;
  }
  static load(name, waypoint_data_list){
    let waypoints = new Waypoints(name);
    waypoint_data_list.forEach((waypoint_data)=>{
      waypoints.addWaypoint(new Waypoint(waypoint_data));
    });
    return waypoints;
  }
  addWaypoint(waypoint) {
    this.add(waypoint);
    waypoint.setWaypoints(this);
  }
  removeWaypoint(waypoint){
    this.remove(waypoint);
  }
  createWaypointFromPosition(position){
    let waypoint_data = new WaypointData(position.x, position.y, position.z, 0, 0, 0);
    let waypoint = new Waypoint(waypoint_data);
    this.addWaypoint(waypoint);
    return waypoint;
  }

  /********************************
   * Group-Model interface methds *
   ********************************/
  getSelectableObjects(){
    return this.children;
  }

}