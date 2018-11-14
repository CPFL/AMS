/**
 * @author Taiga Nishiyama
 * roadsign - 標識
 **/

export class Roadsign extends THREE.Group {
  
  constructor(roadsignData, vector){
    super();
    this.name = 'Roadsign';
    this.data = roadsignData;
    this.vector = vector;
    this.add(vector);
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
    return [this.vector];
  }

  clone(){
    return this.road_structure.cloneVectorBasedObject(this);
  }
  delete(){
    this.parent.remove(this);
  }

}

export class RoadsignData {

  constructor(id, vid, plid, type, link_id){
    this.id = parseInt(id, 10);
    this.vid = parseInt(vid, 10);
    this.plid = parseInt(plid, 10);
    this.type = type;
    this.link_id = parseInt(link_id, 10);
  }
  toCsv() {
    return this.id + ',' + this.vid + ',' + this.plid + ',' + this.type + ',' + this.link_id + '\n';
  }
  getCsvHeader(){
    return 'ID,VID,PLID,Type,LinkID\n';
  }

}
