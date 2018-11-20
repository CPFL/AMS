/**
 * @author Taiga Nishiyama
 * Poledata - ポール
 * Poledata and PoleData(the data class of Pole) is not same. 
 * Poledata と PoleDataは一緒ではありません。
 * - PoledataおよびPoledataDataはシンプルなポールおよびそのデータ構造です。
 * - PoleおよびPoleDataは基本図形クラスで定義されたポールおよびそのデータ構造です(単体では存在されず、他のオブジェクトから参照されます)
 **/

export class Poledata extends THREE.Group {
  
  constructor(poledata_data, pole){
    super();
    this.name = 'Poledata';
    this.data = poledata_data;
    this.pole = pole;
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

export class PoledataData {

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
