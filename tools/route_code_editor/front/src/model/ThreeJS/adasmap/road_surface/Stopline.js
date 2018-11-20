/**
 * @author Taiga Nishiyama
 * 
 **/

export class Stopline extends THREE.Group {
  
  constructor(stoplineData, line){
    super();
    this.name = 'Stopline';
    this.data = stoplineData;
    line.changeDefaultColor(0xff6b6b);
    this.line = line;
    this.add(line);
    this.hilight = false;
    this.road_surface = null;
  }
  setRoadSurface(road_surface){
    this.road_surface = road_surface;
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
    this.road_surface.removeLineBasedObject(this);
  }

}

export class StoplineData {

  constructor(id, lid, tl_id, sign_id, link_id){
    this.id = parseInt(id, 10);
    this.lid = parseInt(lid, 10);
    this.tl_id = parseInt(tl_id, 10);
    this.sign_id = parseInt(sign_id, 10);
    this.link_id = parseInt(link_id, 10);
  }
  toCsv() {
    return this.id + ',' + this.lid + ',' + this.tl_id + ','
    + this.sign_id + ',' + this.link_id + '\n';
  }
  getCsvHeader(){
    return 'ID,LID,TLID,SignID,LinkID\n';
  }

}
