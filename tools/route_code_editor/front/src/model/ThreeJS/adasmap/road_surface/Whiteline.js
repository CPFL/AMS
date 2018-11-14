/**
 * @author Taiga Nishiyama
 * 
 **/

export class Whiteline extends THREE.Group {
  
  constructor(whitelineData, line){
    super();
    this.name = 'WhiteLine';
    this.data = whitelineData;
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

export class WhitelineData {

  constructor(id, lid, width, color, type, link_id){
    this.id = parseInt(id, 10);
    this.lid = parseInt(lid, 10);
    this.width = parseFloat(width);
    this.color = color;
    this.type = parseInt(type, 10);
    this.link_id = parseInt(link_id, 10);
  }
  toCsv() {
    return this.id + ',' + this.lid + ',' + this.width + ',' + this.color + ','
    + this.type + ',' + this.link_id + '\n';
  }
  getCsvHeader(){
    return 'ID,LID,Width,Color,type,LinkID\n';// Warning: 'type' of 't' is low case because of strange specification
  }

}
