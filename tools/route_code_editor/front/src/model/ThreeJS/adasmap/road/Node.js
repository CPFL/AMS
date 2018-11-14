/**
 * @author Taiga Nishiyama
 * Define classes Node, NodeData
 **/

import { Point, PointDataBuilder } from '../geometry/Point';

/**
 * Node
 * let node = new Node(nodeData);
 * // Create new node and new point from position
 * let node = Node.createNodeAndPointFromPosition({x: 0, y: 0, z:0});
 * // Create new node from point
 * let node = Node.createNodeFromPoint(point);
 */
export class Node extends THREE.Group {

  /** 
   * @param {NodeData} nodeData
   * @param {Point} point
   **/
  constructor(nodeData, point) {
    super();
    this.name = 'Node';
    this.data = nodeData;
    point.addUpdateObject(this);
    point.changeDefaultColor(0x0061ff);
    this.point = point;
    this.add(point);
    this.lanes = [];
    this.hilight = false;
    this.road = null;
  }
  /**
   * Create Node without NodeData
   * @param {Point} point 
   */
  static createNodeFromPoint(point, nid) {
    let nodeData = new NodeData(nid, point.data.pid);
    let node = new Node(nodeData, point);
    node.updateRelatedObjects();
    return node;
  }
  /**
   * Create Node without NodeData and Point
   * @param {{x: number, y:number, z:number}} position 
   */
  static createNodeAndPointFromPosition(position, pid, nid, ref) {
    let pointData = new PointDataBuilder().setPid(pid).setBx(position.y).setLy(position.x).setH(position.z).setRef(ref).build();
    let point = new Point(pointData);
    return this.createNodeFromPoint(point, nid);
  }

  /**
   * Add relation of Lane
   * @param {Lane} lane 
   */
  addLane(lane) {
    this.lanes.push(lane);
  }
  /**
   * Remove relation of Lane
   * @param {Lane} lane 
   */
  removeLane(lane) {
    this.lanes = this.lanes.filter((v) => {
      return v != lane;
    });
  }
  /**
   * @param {Road} road 
   */
  setRoad(road){
    this.road = road;
  }

  /**********************************
   * Group model methds *
   **********************************/
  updateRelatedObjects(updatedObjects=[]) {
    // UpdatedObjects(更新済みオブジェクト)にpointが含まれていない時，更新
    if(!updatedObjects.find(obj=> obj === this.point)){
      this.point.updateRelatedObjects();
    }
    this.lanes.forEach((lane) => {
      lane.updateRelatedObjects();
    });
  }
  enableHighlight(color) {
    this.point.enableHighlight(color);
  }

  disableHighlight() {
    this.point.disableHighlight();
  }

  delete(){
    this.road.removeNode(this);
  }

}

/**
 * NodeData
 * @param {number|string} nid 
 * @param {number|string} pid 
 */
export class NodeData {

  constructor(nid, pid) {
    this.nid = parseInt(nid, 10);
    this.pid = parseInt(pid, 10);
  }
  toCsv(){
    return this.nid + ',' + this.pid + '\n';
  }
  getCsvHeader(){
    return 'NID,PID\n';
  }

}
