/**
 * @author Taiga Nishiyama
 * roadsign - 標識
 **/

export class Streetlight extends THREE.Group {
  
  constructor(streetlightData, line){
    super();
    this.name = 'Streetlight';
    this.data = streetlightData;
    this.line = line;
    this.add(line);
    this.add(line.before_point);
    this.add(line.foward_point);
    this.hilight = false;
    this.road_structure = null;
    this.pole = null;
  }
  setPole(pole){
    this.pole = pole;
  }
  setRoadStructure(road_structure){
    this.road_structure = road_structure;
  }

  getSelectableObjects(){
    return [this.line];
  }

  delete(){
    this.parent.remove(this);
  }

}

export class StreetlightData {

  constructor(id, lid, plid, link_id){
    this.id = parseInt(id, 10);
    this.lid = parseInt(lid, 10);
    this.plid = parseInt(plid, 10);
    this.link_id = parseInt(link_id, 10);
  }
  toCsv() {
    return this.id + ',' + this.lid + ',' + this.plid + ',' + this.link_id + '\n';
  }
  getCsvHeader(){
    return 'ID,LID,PLID,LinkID\n';
  }

}
