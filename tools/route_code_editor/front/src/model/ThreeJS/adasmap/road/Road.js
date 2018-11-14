import { Node, Lane, Wayarea, WayareaData, Dtlane, DtlaneData, NodeData } from './index';
import ModelCreateHelper from '../../../../util/ModelCreateHelper';

/**
 * Road
 * @example
 * let road = new Road();
 * let node1 = road.createNode({x:0, y:0, z:0});
 * let {node, line} = road.createNodeAndLane(node1);
 */
export default class Road extends THREE.Group {

  constructor() {
    super();
    this.name = 'Road';
    // Lanes
    this.lanes = new THREE.Group();
    this.add(this.lanes);
    this.lanes.getSelectableObjects = function () { return this.children; };
    // Nodes
    this.nodes = new THREE.Group();
    this.add(this.nodes);
    this.nodes.getSelectableObjects = function () { return this.children.map(node => node.point); };
    // Wayareas
    this.wayareas = new THREE.Group();
    this.add(this.wayareas);
    this.wayareas.getSelectableObjects = () => {
      return this.wayareas.children.reduce((arr, wayarea)=> arr.concat(wayarea.area.getSelectableObjects()), []);
    };
    // Dtlanes
    this.dtlanes = new THREE.Group();
    this.add(this.dtlanes);
    this.dtlanes.getSelectableObjects = () => {
      return this.dtlanes.children.map(dtlane => dtlane.point);
    };
    this.max_id = {};
    this.lane_max_length = 1;
  }
  /**
   * @param {object} models
   * {
   *   points: array of point
   *   lines: array of line,
   * }
   * @param {object} data
   * {
   *   node_data_list: array of node_data,
   *   lane_data_list: array of lane_data,
   * }
   */
  static load(models, data) {
    let road = new Road();
    // Load Lane and Node
    if (models.points && data.node_data_list && data.lane_data_list) {
      let nodes = [];
      data.node_data_list.forEach((node_data) => {
        let point = models.points.find((point) => point.data.pid === node_data.pid);
        if (!point) {
          console.error('Not found point data(pid:' + node_data.pid + ')');
          return;
        }
        let node = new Node(node_data, point);
        nodes.push(node);
        road.addObject(node, 'node', 'nid');
      });
      data.lane_data_list.forEach((lane_data) => {
        let before_node = nodes.find(node => node.data.nid === lane_data.bnid);
        let foward_node = nodes.find(node => node.data.nid === lane_data.fnid);
        if (before_node == null || foward_node == null) {
          console.error('Not found Node');
          return;
        }
        let lane = new Lane(lane_data, before_node, foward_node);
        road.addObject(lane, 'lane', 'lnid');
      });
    }
    // Load Wayareas
    if (models.areas && data.wayarea_data_list) {
      data.wayarea_data_list.forEach((wayarea_data) => {
        let area = models.areas.find(area => area.data.aid === wayarea_data.aid);
        let wayarea = new Wayarea(wayarea_data, area);
        road.addObject(wayarea, 'wayarea', 'waid');
      });
    }
    // Load Dtlanes
    if (models.points && data.dtlane_data_list) {
      data.dtlane_data_list.forEach((dtlane_data) => {
        let point = models.points.find(p => p.data.pid === dtlane_data.pid);
        let dtlane = new Dtlane(dtlane_data, point);
        road.addObject(dtlane, 'dtlane', 'did');
      });
    }
    return road;
  }
  addObject(object, name, id_name){
    if(this.max_id[name] === undefined){
      this.max_id[name] = 0;
    }
    if(object.data[id_name] > this.max_id[name]){
      this.max_id[name] = object.data[id_name];
    }
    object.setRoad(this);
    this[name+'s'].add(object);
  }

  removeNode(node) {
    // Before Delete the node, Delete lanes that depend on the node.
    node.lanes.forEach(lane=>lane.delete());
    this.nodes.remove(node);
  }
  removeLane(lane) {
    this.lanes.remove(lane);
  }
  removeDtlane(dtlane){
    this.dtlanes.remove(dtlane);
  }

  createDtlane(position) {
    let adasmap = this.parent;
    let did = this.incrementMaxId('dtlane');
    let point = ModelCreateHelper.createPointFromPosition(adasmap, position);
    let dtlane_data = new DtlaneData(did, 0, point.data.pid, 0, 0, 0, 0, 0, 0, 0);
    let dtlane = new Dtlane(dtlane_data, point);
    this.addObject(dtlane, 'dtlane', 'did');
    return dtlane;
  }

  /**
   * Create New Node from Position
   * @param {{x: number, y: number, z:number}} position 
   */
  createNode(position) {
    let adasmap = this.parent;
    let point = ModelCreateHelper.createPointFromPosition(adasmap, position);
    return this.createNodeFromPoint(point);
  }
  createNodeFromPoint(point) {
    let nid = this.incrementMaxId('node');
    let node_data = new NodeData(nid, point.data.pid);
    let node = new Node(node_data, point);
    this.addObject(node, 'node', 'nid');
    return node;
  }
  /**
   * Create new lane
   * @param {Node} before_node 
   * @param {Node} foward_node 
   * @return {Lane}
   */
  createLane(before_node, foward_node) {
    let lnid = this.incrementMaxId('lane');
    let lane = Lane.createLane(before_node, foward_node, lnid);
    this.addObject(lane, 'lane', 'lnid');
    return lane;
  }
  /**
   * Create new node and new lane
   * The position of the new node is near before_node(passed node)
   * @param {Node} before_node
   * @return {{Node, Lane}} {node, lane}
   */
  createNodeAndLane(before_node) {
    // before_nodeをfoward_nodeとしている前のlaneがある場合は延長する方向にlaneとnodeを作成
    let newPoint;
    if(before_node.lanes.find(lane => lane.foward_node == before_node)){
      let before_lane = before_node.lanes.find(lane => lane.foward_node == before_node);
      let point1 = before_lane.before_node.point;
      let point2 = before_node.point;
      newPoint = ModelCreateHelper.createPointAlongWithExsiting(this.parent, point1, point2);
    }else{
      let newPosition = before_node.point.position.clone().add(new THREE.Vector3(1, 0, 0));
      newPoint = ModelCreateHelper.createPointFromPosition(this.parent, newPosition);
    }
    return this.createNodeAndLaneFromPoint(before_node, newPoint);
  }
  createNodeAndLaneFromPoint(before_node, newPoint){
    let foward_node = this.createNodeFromPoint(newPoint);
    let lane = this.createLane(before_node, foward_node);
    return { node: foward_node, lane };
  }
  /**
   * Create new node and new lane with before_node & new node position
   * @param {Node} before_node
   * @param {THREE.Vector3} newPosition
   * @return {{Node, Lane}} {node, lane}
   */
  createNodeAndLaneWithPosition(before_node, newPosition) {
    if (!before_node) {
      console.error('Please select the exsiting node');
    }
    // Check the passed node is belongs to this road
    if (this.nodes.children.includes(before_node)) {
      let foward_node = this.createNode(newPosition);
      let lane = this.createLane(before_node, foward_node);
      return { node: foward_node, lane };
    } else {
      console.error('Passed node is wrong');
    }
  }
  createAreaBasedObjectFromPosition(ModelClass, position){
    let area = ModelCreateHelper.createAreaFromPosition(this.parent, position);
    if(ModelClass === Wayarea){
      let waid = this.incrementMaxId('wayarea');
      let wayarea_data = new WayareaData(waid, area.data.aid);
      let wayarea = new Wayarea(wayarea_data, area);
      this.addObject(wayarea, 'wayarea', 'waid');
      return wayarea;
    }
  }

  // Check and Update lanes to follow the rule of max length
  checkAndUpdateLanes(lanes) {
    lanes.forEach((lane) => {
      if (lane.data.span > this.lane_max_length) {
        this.splitLane(lane);
      }
    });
  }
  // Split lane to follow the rules of max length
  // If the lane span is 2.5[m], the lane is divided into 3 lanes 1[m], 1[m], 0.5[m].
  // レーンの最大長制限に沿ってレーンを分割する
  // 2.5メートルの場合は，1メートル，1メートル，0.5メートルという3つのレーンに分割をする
  splitLane(lane) {
    let length = lane.data.span;
    // 分割数と最後のlaneの長さ
    // 例：
    // 2.5m -> 1m(既存lane), 1m, 0.5m に分割
    // new_lane_num = 2, fraction_span = 0.5;
    let new_lane_num = Math.ceil(length / this.lane_max_length) - 1;
    let fraction_span = length % this.lane_max_length;
    let foward_lanes = lane.getFowardLanes();

    if(new_lane_num > 0){
      // Normalized lane vector to 1[m]
      // レーンのベクトルを1メートルにノーマライズしたベクトル
      let lane_vector_normalize = lane.foward_node.point.position.clone().sub(lane.before_node.point.position).normalize();
      // 既存のlaneのfoward_nodeの位置を変更
      let before_node = lane.before_node;
      let foward_node = lane.foward_node;
      foward_node.point.position.copy(before_node.point.position.clone().add(lane_vector_normalize));
      lane.updateRelatedObjects();
      // 1mの新しいlaneを追加
      // new_lane_num = 1の時，端数の長さのlaneのみ追加するため1mのlaneは追加しない
      for (let i = 0; i < new_lane_num - 1; i++) {
        let newPosition = foward_node.point.position.clone().add(lane_vector_normalize);
        let { node } = this.createNodeAndLaneWithPosition(foward_node, newPosition);
        foward_node = node;
      }
      // 端数の長さの終端laneを追加
      let newPosition = foward_node.point.position.clone().add(lane_vector_normalize.multiplyScalar(fraction_span));
      let { lane: new_lane, node } = this.createNodeAndLaneWithPosition(foward_node, newPosition);
      foward_lanes.forEach(foward_lane =>{
        foward_lane.changeBeforeNode(node);
        foward_lane.setLaneRelation();
      });
      return new_lane;
    }else{
      return lane;
    }
  }
  // max_id.name を1増加し、その値を返す
  incrementMaxId(name){
    if(this.max_id[name] === undefined){
      this.max_id[name] = 0;
    }
    this.max_id[name] += 1;
    return this.max_id[name];
  }

  /********************************
   * Group-Model interface methds *
   ********************************/
  getRoadObjectGroups(){
    return [this.lanes, this.nodes, this.wayareas, this.dtlanes];
  }

}
