/**
 * @author Taiga Nishiyama
 * 
 **/

export class Curb extends THREE.Group {
  
  constructor(curbData, line){
    super();
    this.name = 'Curb';
    this.data = curbData;
    line.changeDefaultColor(0xff2bff);
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

export class CurbData {

  constructor(id, lid, height, width, dir, link_id){
    this.id = parseInt(id, 10);
    this.lid = parseInt(lid, 10);
    this.height = parseFloat(height);
    this.width = parseFloat(width);
    this.dir = parseInt(dir, 10);
    this.link_id = parseInt(link_id, 10);
  }
  toCsv() {
    return this.id + ',' + this.lid + ',' + this.height + ','
    + this.width + ',' + this.dir + ',' + this.link_id + '\n';
  }
  getCsvHeader(){
    return 'ID,LID,Height,Width,Dir,LinkID\n';
  }
  inherit(curbData){
    this.height = curbData.height;
    this.width = curbData.width;
    this.dir = curbData.dir;
  }

}
