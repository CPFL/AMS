/**
 * @author Taiga Nishiyama
 * curvemirror - カーブミラー
 **/

export class Curvemirror extends THREE.Group {
  
  constructor(curvemirror_data, vector){
    super();
    this.name = 'Curvemirror';
    this.data = curvemirror_data;
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

  clone(){
    return this.road_structure.cloneVectorBasedObject(this);
  }

  getSelectableObjects(){
    return [this.vector];
  }

  delete(){
    this.parent.remove(this);
  }

}

export class CurvemirrorData {

  constructor(id, vid, plid, type, link_id){
    this.id = parseInt(id, 10);
    this.vid = parseInt(vid, 10);
    this.plid = parseInt(plid, 10);
    this.type = parseInt(type, 10);
    this.link_id = parseInt(link_id, 10);
  }
  toCsv() {
    return this.id + ',' + this.vid + ',' + this.plid + ',' + this.type + ',' + this.link_id + '\n';
  }
  getCsvHeader(){
    return 'ID,VID,PLID,Type,LinkID\n';
  }

}
