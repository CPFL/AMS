/**
 * @author Taiga Nishiyama
 * Define classes Node, NodeData
 **/

export class Waypoint extends THREE.Mesh {

  constructor(waypointData) {
    let geometry = new THREE.Geometry();
    geometry.vertices.push(new THREE.Vector3(-0.2, 0.05, 0.05));
    geometry.vertices.push(new THREE.Vector3(-0.2, -0.05, 0.05));
    geometry.vertices.push(new THREE.Vector3(0.1, 0.05, 0.05));
    geometry.vertices.push(new THREE.Vector3(0.1, -0.05, 0.05));
    geometry.vertices.push(new THREE.Vector3(-0.2, 0.05, -0.05));
    geometry.vertices.push(new THREE.Vector3(-0.2, -0.05, -0.05));
    geometry.vertices.push(new THREE.Vector3(0.1, 0.05, -0.05));
    geometry.vertices.push(new THREE.Vector3(0.1, -0.05, -0.05));
    geometry.vertices.push(new THREE.Vector3(0.1, 0.09, 0.09));
    geometry.vertices.push(new THREE.Vector3(0.1, -0.09, 0.09));
    geometry.vertices.push(new THREE.Vector3(0.1, -0.09, -0.09));
    geometry.vertices.push(new THREE.Vector3(0.1, 0.09, -0.09));
    geometry.vertices.push(new THREE.Vector3(0.2, 0, 0));
    geometry.faces.push(new THREE.Face3(0, 1, 2));
    geometry.faces.push(new THREE.Face3(1, 3, 2));
    geometry.faces.push(new THREE.Face3(1, 7, 3));
    geometry.faces.push(new THREE.Face3(1, 5, 7));
    geometry.faces.push(new THREE.Face3(0, 4, 1));
    geometry.faces.push(new THREE.Face3(1, 4, 5));
    geometry.faces.push(new THREE.Face3(2, 4, 0));
    geometry.faces.push(new THREE.Face3(2, 6, 4));
    geometry.faces.push(new THREE.Face3(5, 4, 7));
    geometry.faces.push(new THREE.Face3(7, 4, 6));
    geometry.faces.push(new THREE.Face3(8, 11, 9));
    geometry.faces.push(new THREE.Face3(9, 11, 10));
    geometry.faces.push(new THREE.Face3(12, 8, 9));
    geometry.faces.push(new THREE.Face3(12, 11, 8));
    geometry.faces.push(new THREE.Face3(12, 10, 11));
    geometry.faces.push(new THREE.Face3(12, 9, 10));
    geometry.computeFaceNormals();
    super(geometry, new THREE.MeshBasicMaterial({ color: 0xcccccc }));
    this.name = 'Waypoint';
    this.data = waypointData;
    this.rotation.z = this.data.yaw;
    this.position.copy(new THREE.Vector3(this.data.x, this.data.y, this.data.z));
    this.hilight = false;
    this.waypoints = null;
  }

  setWaypoints(waypoints){
    this.waypoints = waypoints;
  }
  setVal(params){
    let name = params.name;
    let val = params.val;
    if(name === 'x' || name === 'y' || name === 'z'){
      this.position[name] = val;
    }else if(name === 'yaw'){
      this.rotation.z = val;
    }
    this.updateRelatedObjects();
  }

  /**********************************
   * Object model interface methds *
   **********************************/
  updateRelatedObjects() {
    this.position.x = Math.round(this.position.x * 10000)/10000;
    this.position.y = Math.round(this.position.y * 10000)/10000;
    this.position.z = Math.round(this.position.z * 10000)/10000;
    this.data.x = this.position.x;
    this.data.y = this.position.y;
    this.data.z = this.position.z;
    this.data.yaw = this.rotation.z;
  }
  enableHighlight(color) {
    let material_color = color || 0xf6ff4c;
    this.material = new THREE.MeshBasicMaterial({ color: material_color });
    this.hilight = true;
  }
  disableHighlight() {
    this.material = new THREE.LineBasicMaterial({ color: 0xcccccc });
    this.hilight = false;
  }
  delete(){
    this.waypoints.removeWaypoint(this);
  }

}

export class WaypointData {

  constructor(x, y, z, yaw, velocity, change_flag) {
    this.x = parseFloat(x);
    this.y = parseFloat(y);
    this.z = parseFloat(z);
    this.yaw =  parseFloat(yaw);
    this.velocity = parseFloat(velocity);
    this.change_flag = parseInt(change_flag, 10);
  }
  toCsv(){
    return this.x + ',' + this.y + ',' + this.z + ',' + this.yaw + ',' + this.velocity + ',' + this.change_flag + '\n';
  }
  getCsvHeader(){
    return 'x,y,z,yaw,velocity,change_flag\n';
  }

}
