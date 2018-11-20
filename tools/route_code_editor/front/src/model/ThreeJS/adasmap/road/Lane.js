/**
 * @author Taiga Nishiyama
 * Define classes Lane, LaneData, LaneDataBuilder
 **/

/** 
 * Lane
 * @example
 * let lane = new Lane(laneData, before_node, foward_node);
 * // Create new lane 
 * let lane = Lane.createNewLane(before_node, fowar_node)
 **/
export class Lane extends THREE.Line {

  /**
   * @param {LaneData} laneData 
   * @param {Node} before_node 
   * @param {Node} foward_node 
   */
  constructor(laneData, before_node, foward_node) {
    let material = new THREE.LineBasicMaterial({ color: 0x00e2ea });
    let geometry = new THREE.Geometry();
    geometry.vertices.push(before_node.point.position, foward_node.point.position);
    super(geometry, material);
    this.defaultColor = 0x00e2ea;
    this.name = 'Lane';
    this.data = laneData;
    this.before_node = before_node;
    this.foward_node = foward_node;
    before_node.addLane(this);
    foward_node.addLane(this);
    this.hilight = false;
    this.road = null;
  }
  static createLane(before_node, foward_node, lnid) {
    let span = Math.round(before_node.point.position.distanceTo(foward_node.point.position) * 1000)/1000;
    let laneData = new LaneDataBuilder().setLnid(lnid).setBnid(before_node.data.nid).setFnid(foward_node.data.nid).setSpan(span).build();
    let lane = new Lane(laneData, before_node, foward_node);
    lane.setLaneRelation();
    return lane;
  }

  /**
   * Change Before Node
   * @param {Node} newNode 
   */
  changeBeforeNode(newNode) {
    let index = this.geometry.vertices.indexOf(this.before_node.point.position);
    this.geometry.vertices.splice(index, 1);
    this.geometry.vertices.push(newNode.point.position);
    this.before_node.removeLane(this);
    this.before_node = newNode;
    this.before_node.addLane(this);
    this.data.bnid = newNode.data.nid;
  }
  /**
   * Change Foward Node
   * @param {Node} newNode 
   */
  changeFowardNode(newNode) {
    let index = this.geometry.vertices.indexOf(this.foward_node.point.position);
    this.geometry.vertices.splice(index, 1);
    this.geometry.vertices.push(newNode.point.position);
    this.foward_node = newNode;
    this.data.fnid = newNode.data.nid;
  }
  /**
   * Set BLID, FLID automatically
   * BLID, FLIDを自動で設定する
   */
  setLaneRelation(){
    /*
     * 1. laneのBLID,FLIDの設定判定
     * lane(lane1)のbefore_lane(注1)が一つのみの時，laneのBLIDをbefore_laneのIDに設定
     * 注1 lane1のbefore_nodeをfoward_nodeとするlane
     * lane(lane1)のfoward_lane(注2)が一つのみの時，laneのFLIDをfoward_laneのIDに設定
     * 注2 lane1のfoward_nodeをbefore_nodeとするlane
     */
    let before_lanes = this.getBeforeLanes();
    if(before_lanes.length == 1){
      this.data.blid = before_lanes[0].data.lnid;
    }
    let foward_lanes = this.getFowardLanes();
    if(foward_lanes.length == 1){
      this.data.flid = foward_lanes[0].data.lnid;
    }
    /*
     * 2. laneの前後のlaneが自身をBLIF,FLIDと設定すべきかの判定
     * lane(lane1)のbefore_lanesのfoward_lanesがlane1のみの時，before_lanesのFLIDをlane1のIDに設定
     * lane(lane1)のfoward_lanesのbefore_lanesがlane1のみの時，foward_lanesのBLIDをlane1のIDに設定
     */
    if(before_lanes.length > 0){
      let isUniqueFowardLane = before_lanes[0].getFowardLanes().length == 1;
      if(isUniqueFowardLane){
        before_lanes.forEach(before_lane => before_lane.data.flid = this.data.lnid);
      }
    }
    if(foward_lanes.length > 0){
      let isUniqueBeforeLane = foward_lanes[0].getBeforeLanes().length == 1;
      if(isUniqueBeforeLane){
        foward_lanes.forEach(foward_lane => foward_lane.data.blid = this.data.lnid);
      }
    }
  }
  /**
   * @param {Road} road 
   */
  setRoad(road){
    this.road = road;
  }

  /**
   * 前方のlanesを取得
   * 前方のlanes: laneのfoward_nodeをbefore_nodeとするlanes
   * 注意：flidで設定されているlaneを取得するわけではありません
   */
  getFowardLanes(){
    let foward_node = this.foward_node;
    return foward_node.lanes.filter(lane => lane.before_node === foward_node);
  }
  /**
   * 後方のlanesを取得
   * 後方のlanes: laneのbefore_nodeをfoward_nodeとするlanes
   * 注意：blidで設定されているlaneを取得するわけではありません
   */
  getBeforeLanes(){
    let before_node = this.before_node;
    return before_node.lanes.filter(lane => lane.foward_node === before_node);
  }

  /**********************************
   * Visible model interface methds *
   **********************************/
  updateRelatedObjects() {
    this.geometry.verticesNeedUpdate = true;
    this.geometry.boundingSphere = null;
    this.data.span = Math.round(this.before_node.point.position.distanceTo(this.foward_node.point.position) * 1000)/1000;
  }
  changeDefaultColor(color) {
    this.defaultColor = color;
    if(this.hilight === false){
      this.material = new THREE.LineBasicMaterial({ color: this.defaultColor });
    }
  }
  enableHighlight(color) {
    let material_color = color || 0xf6ff4c;
    this.material = new THREE.LineBasicMaterial({ color: material_color });
    let vector = this.foward_node.point.position.clone().sub(this.before_node.point.position);
    let arrow_length = this.foward_node.point.position.distanceTo(this.before_node.point.position) * 1/2;
    let arrow_dir = vector.clone().normalize();
    // ラインとZ軸に直角に交わるベクトルを求める
    let cross_vector = new THREE.Vector3().crossVectors(arrow_dir, new THREE.Vector3(0, 0, 1)).normalize();
    let origin = this.before_node.point.position.clone().add(vector.multiplyScalar(1/4)).add(cross_vector.multiplyScalar(0.1));
    var arrow = new THREE.ArrowHelper(arrow_dir, origin, arrow_length, 0x999999);
    this.arrow = arrow;
    this.add(arrow);
    this.hilight = true;
  }
  disableHighlight() {
    this.material = new THREE.LineBasicMaterial({ color: this.defaultColor });
    this.remove(this.arrow);
    this.hilight = false;
  }
  delete(){
    this.road.removeLane(this);
  }
  changeDirection(){
    let swap;
    swap = this.before_node;
    this.before_node = this.foward_node;
    this.foward_node = swap;
    this.data.bnid = this.before_node.data.nid;
    this.data.fnid = this.foward_node.data.nid;
    this.disableHighlight();
    this.enableHighlight();
  }

}
/**
 * LaneData
 */
export class LaneData {

  /**
   * 
   * @param {number|string} lnid - Lane ID (int)
   * @param {number|string} did - Center liner id 中心線形ID (int)
   * @param {number|string} blid - Lane ID of begin, 手前のレーンID 始点データは0 (int)
   * @param {number|string} flid - Lane ID of foward, 次のレーンID 終点データは0 (int)
   * @param {number|string} bnid - Node ID of start point, 始点ノードID (int)
   * @param {number|string} fnid - Node ID of end point, 終点ノードID (int)
   * @param {number|string} jct - Bifurcation merge pattern, 分岐合流パターン (int)
   *                       通常区間(0),左へ合流(1), 右へ合流(2), 左から合流(3), 右から合流(4)
   * @param {number|string} blid2 - 合流ID2． BLID以外にこのレーンに合流するレーンID (int)
   * @param {number|string} blid3 - 合流ID3． BLID以外にこのレーンに合流するレーンID (int)
   * @param {number|string} blid4 - 合流ID4． BLID以外にこのレーンに合流するレーンID (int)
   * @param {number|string} flid2 - 分岐ID2． FLID以外にこのレーンから分岐するレーンID (int)
   * @param {number|string} flid3 - 分岐ID3． FLID以外にこのレーンから分岐するレーンID (int)
   * @param {number|string} flid4 - 分岐ID4．FLID以外にこのレーンから分岐するレーンID (int)
   * @param {number|string} clossid - このレコードが交差点に含まれる場合そのID．通常は0(int)
   * @param {number|string} span - 区間距離, この線分の長さ(double)
   * @param {number|string} lcnt - 車線数 交差点内は0(int)
   * @param {number|string} lno - レーン番号．左から1 交差点内は0(int)
   * @param {number|string} lanetype - レーン種別．0:直進レーン 1:左折レーン 2:右折レーン(int)
   * @param {number|string} limitvel - Legal speed 法定速度(double)
   * @param {number|string} refvel - 目標速度 走行時の想定速度(現在は、法定速度以内の最大速度を設定)(double)
   * @param {number|string} roadsecid - 道路区間ID．道路区間ID 交差点内は0 隣り合うレーンは、通常同じ区間IDを持つ(int)
   * @param {number|string} lanechgfg - 車線変更可/不可．0:可 1:不可 (交差点内は0)(int)
   * @param {number|string} linkwaid - 属する走行エリアID(int)
   */
  constructor(lnid, did, blid, flid, bnid, fnid, jct, blid2, blid3, blid4, flid2, flid3, flid4, clossid, span, lcnt, lno, lanetype, limitvel, refvel, roadsecid, lanechgfg, linkwaid) {
    this.lnid = parseInt(lnid, 10);
    this.did = parseInt(did, 10);
    this.blid = parseInt(blid, 10);
    this.flid = parseInt(flid, 10);
    this.bnid = parseInt(bnid, 10);
    this.fnid = parseInt(fnid, 10);
    this.jct = parseInt(jct, 10);
    this.blid2 = parseInt(blid2, 10);
    this.blid3 = parseInt(blid3, 10);
    this.blid4 = parseInt(blid4, 10);
    this.flid2 = parseInt(flid2, 10);
    this.flid3 = parseInt(flid3, 10);
    this.flid4 = parseInt(flid4, 10);
    this.clossid = parseInt(clossid, 10);
    this.span = parseFloat(span);
    this.lcnt = parseInt(lcnt, 10);
    this.lno = parseInt(lno, 10);
    this.lanetype = parseInt(lanetype, 10);
    this.limitvel = parseFloat(limitvel);
    this.refvel = parseFloat(refvel);
    this.roadsecid = parseInt(roadsecid, 10);
    this.lanechgfg = parseInt(lanechgfg, 10);
    this.linkwaid = parseInt(linkwaid, 10);
  }
  toCsv() {
    return this.lnid + ',' + this.did + ',' + this.blid + ',' + this.flid + ','
           + this.bnid + ',' + this.fnid+ ',' + this.jct+ ','
           + this.blid2 + ',' + this.blid3 + ',' + this.blid4 + ','
           + this.flid2 + ',' + this.flid3 + ',' + this.flid4 + ','
           + this.clossid + ',' + this.span + ',' + this.lcnt + ','
           + this.lno + ',' + this.lanetype + ',' + this.limitvel + ','
           + this.refvel + ',' + this.roadsecid + ',' + this.lanechgfg + ','
           + this.linkwaid + '\n';
  }
  getCsvHeader(){
    return 'LnID,DID,BLID,FLID,BNID,FNID,JCT,BLID2,BLID3,BLID4,FLID2,FLID3,FLID4,ClossID,Span,LCnt,Lno,LaneType,LimitVel,RefVel,RoadSecID,LaneChgFG,LinkWAID\n';
  }

}

/**
 * LaneDataBuilder
 * @example
 * let laneData = new LaneDataBuilder().setLnid(1).setBnid(1).setFnid(2).build();
 **/
export class LaneDataBuilder {

  constructor() {
    this.lnid = null;
    this.did = 0;
    this.blid = 0;
    this.flid = 0;
    this.bnid = null;
    this.fnid = null;
    this.jct = 0;
    this.blid2 = 0;
    this.blid3 = 0;
    this.blid4 = 0;
    this.flid2 = 0;
    this.flid3 = 0;
    this.flid4 = 0;
    this.clossid = 0;
    this.span = null;
    this.lcnt = 1;
    this.lno = 1;
    this.lanetype = 0;
    this.limitvel = 30;
    this.refvel = 30;
    this.roadsecid = 0;
    this.lanechgfg = 0;
    this.linkwaid = 0;
  }
  setLnid(lnid) {
    this.lnid = lnid;
    return this;
  }
  setBnid(bnid) {
    this.bnid = bnid;
    return this;
  }
  setFnid(fnid) {
    this.fnid = fnid;
    return this;
  }
  setSpan(span) {
    this.span = span;
    return this;
  }
  build() {
    return new LaneData(this.lnid, this.did, this.blid, this.flid, this.bnid, this.fnid, this.jct, this.blid2, this.blid3, this.blid4, this.flid2, this.flid3, this.flid4, this.clossid, this.span, this.lcnt, this.lno, this.lanetype, this.limitvel, this.refvel, this.roadsecid, this.lanechgfg, this.linkwaid);
  }

}
