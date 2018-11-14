/**
 * @author Taiga Nishiyama
 * roadedge - 道路縁
 **/

export class Roadedge extends THREE.Group {
  
  constructor(roadedgeData, line){
    super();
    this.name = 'Roadedge';
    this.data = roadedgeData;
    line.changeDefaultColor(0x96e800);
    this.line = line;
    this.add(line);
    this.hilight = false;
    this.road_shape = null;
  }
  setRoadShape(road_shape){
    this.road_shape = road_shape;
  }

  /**********************************
   * Visible model interface methds *
   **********************************/
  updateRelatedObjects() {
    this.line.updateRelatedObjects();
  }
  enableHighlight(color) {
    this.line.enableHighlight(color);
  }
  disableHighlight() {
    this.line.disableHighlight();
  }
  delete(){
    this.road_shape.removeLineBasedObject(this);
  }

}

export class RoadedgeData {

  constructor(id, lid, link_id){
    this.id = parseInt(id, 10);
    this.lid = parseInt(lid, 10);
    this.link_id = parseInt(link_id, 10);
  }
  toCsv() {
    return this.id + ',' + this.lid + ',' + this.link_id + '\n';
  }
  getCsvHeader(){
    return 'ID,LID,LinkID\n';
  }

}
