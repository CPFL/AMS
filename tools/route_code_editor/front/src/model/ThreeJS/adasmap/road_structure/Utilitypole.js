/**
 * @author Taiga Nishiyama
 * Utilitypole - 電柱
 **/

export class Utilitypole extends THREE.Group {
  
  constructor(utilitypole_data, pole){
    super();
    this.name = 'Utilitypole';
    this.data = utilitypole_data;
    this.pole = pole;
    pole.changeDefaultColor(0x00d4ff);
    this.add(pole);
    this.hilight = false;
    this.road_structure = null;
  }
  setRoadStructure(road_structure){
    this.road_structure = road_structure;
  }

  getSelectableObjects(){
    return this.pole;
  }
  clone(){
    return this.road_structure.clonePoleBasedObject(this);
  }
  delete(){
    this.parent.remove(this);
  }

}

export class UtilitypoleData {

  constructor(id, plid, link_id){
    this.id = parseInt(id, 10);
    this.plid = parseInt(plid, 10);
    this.link_id = parseInt(link_id, 10);
  }
  toCsv() {
    return this.id + ',' + this.plid + ',' + this.link_id + '\n';
  }
  getCsvHeader(){
    return 'ID,PLID,LinkID\n';
  }

}
