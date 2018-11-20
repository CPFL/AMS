import { Line, Point } from './index';
import ModelCreateHelper from '../../../../util/ModelCreateHelper';
/**
 * @author Taiga Nishiyama
 * 
 **/

export class Area extends THREE.Group {

  /**
   * @param {AreaData} areaData 
   * @param {array of Line} lines (start_lineからend_lineまで順に格納されることを保証する)
   */
  constructor(areaData, lines) {
    super();
    this.data = areaData;
    this.points = []; //{array<Point>}
    this.lines = []; // {array<Line>}
    this.start_line = null; // {Line}
    this.end_line = null; // {Line}
    this.defaultColor = 0xffffff;
    this.hilight = false;
    this.boundingMesh = null;
    
    // Initialize
    let points = []; // {array<Point>}
    // 最初のラインの始点のポイントを追加
    this.start_line = lines.find(l => l.data.lid === areaData.slid);
    points.push(this.start_line.before_point);
    // すべてのラインの終点のポイントを追加
    lines.forEach(line => {
      points.push(line.foward_point);
    });
    this.end_line = lines.find(l => l.data.lid === areaData.elid);
    this.end_line.enableDash();
    this.lines = lines;
    this.points = points;
    points.forEach(p =>{
      p.addUpdateObject(this);
      this.add(p);
    });
    lines.forEach(l => {
      l.addUpdateObject(this);
      this.add(l);
    });
  }

  getSelectableObjects(){
    return this.children.filter(obj => obj instanceof Line || obj instanceof Point);
  }

  extendLine(adasmap){
    //
    //       Before             After
    //      p-----p            p-----p
    //      |     |           /      |
    //   el |     |      el  /       |
    //      |     |         /        |
    //      p2----p1     np----p2----p1
    //         bl                 bl
    // 
    //  bl: before_line, el: end_line
    //  p1: point1, p2: point2, np: new_point
    //
    let before_line = this.end_line.before_line;
    let point1 = before_line.before_point;
    let point2 = before_line.foward_point;
    let new_point = ModelCreateHelper.createPointAlongWithExsiting(adasmap, point1, point2);
    new_point.addUpdateObject(this);
    let new_end_line = ModelCreateHelper.createLine(adasmap, this.end_line, null, new_point, this.start_line.before_point);
    new_end_line.addUpdateObject(this);
    this.changeEndLine(new_end_line);
    return new_end_line;
  }

  // - Areaは閉じた連続線で表現するため、end_lineのfoward_point(終点)は常にstart_lineのbefore_point(始点)となる
  changeEndLine(new_end_line){
    // 現在のend_lineの終点を新しいend_lineの始点に変更し、実線に変更
    this.end_line.changeFowardPoint(new_end_line.before_point);
    this.end_line.disableDash();
    this.end_line = new_end_line;
    this.end_line.enableDash();
    this.lines.push(new_end_line);
    this.add(new_end_line);
    this.points.push(new_end_line.before_point);
    this.add(new_end_line.before_point);
    this.changeDefaultColor(this.parent.defaultColor);
    this.updateRelatedObjects();
  }

  /* TODO
  splitLine(adasmap, line){
    // Before (Split line)
    // before_line ->       line         -> foward_line
    // After  (Insert new_line)
    // before_line -> line ---> new_line -> foward_line
    let midway_position = (line.before_point.positon.clone()).add(line.foward_point.position).addScaler(1/2);
    let midway_point = ModelCreateHelper.createPointFromPosition(adasmap, midway_position);
  }
  */


  /**********************************
   * Object3D model interface methds *
   **********************************/
  updateRelatedObjects() {
    this.data.elid = this.end_line.data.lid;
    this.data.slid = this.start_line.data.lid;
    this.enableHighlight();
  }
  enableHighlight(/*color*/) {
    this.disableHighlight();
    let box = new THREE.Box3().setFromObject(this);
    let expand_min = new THREE.Vector3(box.min.x - 0.1, box.min.y - 0.1, box.min.z - 0.1);
    let expand_max = new THREE.Vector3(box.max.x + 0.1, box.max.y + 0.1, box.max.z + 0.1);
    box.set(expand_min, expand_max);
    this.boundingMesh = new THREE.Box3Helper(box, 0xf6ff4c);
    this.add(this.boundingMesh);
  }
  disableHighlight() {
    if(this.boundingMesh != null){
      this.remove(this.boundingMesh);
      this.boundingMesh = null;
    }
  }
  changeDefaultColor(color) {
    this.defaultColor = color;
    this.children.filter(obj => obj instanceof Line || obj instanceof Point).forEach(obj => obj.changeDefaultColor(this.defaultColor));
  }
  changeDefaultPointSize(size){
    this.points.forEach(point => point.changeDefaultSize(size));
  }

}

export class AreaData {

  constructor(aid, slid, elid){
    this.aid = parseInt(aid, 10);
    this.slid = parseInt(slid, 10);
    this.elid = parseInt(elid, 10);
  }
  toCsv() {
    return this.aid + ',' + this.slid + ',' + this.elid + '\n';
  }
  getCsvHeader(){
    return 'AID,SLID,ELID\n';
  }

}
