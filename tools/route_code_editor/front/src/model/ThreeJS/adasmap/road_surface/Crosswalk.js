/**
 * @author Taiga Nishiyama
 * 路面 - 横断歩道
 * Road surface - Crosswalk
 **/
export class Crosswalk extends THREE.Group{

  constructor(crosswalkData, area){
    super();
    this.name = 'Crosswalk';
    this.data = crosswalkData;
    this.area = area;
    if(this.data.type === 0){
      this.area.changeDefaultColor(0xcccccc);
      this.defaultColor = 0xcccccc;
    }else if(this.data.type === 1){
      this.area.changeDefaultColor(0xffffff);
      this.defaultColor = 0xffffff;
    }
    this.add(area);
    this.hilight = false;
    this.road_shape = null;
  }
  setRoadSurface(road_surface){
    this.road_surface = road_surface;
  }
  delete(){
    this.parent.remove(this);
  }


}
export class CrosswalkData {

  constructor(id, aid, type, bdid, link_id) {
    this.id = parseInt(id, 10);
    this.aid = parseInt(aid, 10);
    this.type = parseInt(type, 10);
    this.bdid = parseInt(bdid, 10);
    this.link_id = parseInt(link_id, 10);
  }
  toCsv() {
    return this.id + ',' + this.aid + ',' + this.type + ',' +this.bdid + ',' +this.link_id + '\n';
  }
  getCsvHeader(){
    return 'ID,AID,Type,BdID,LinkID\n';
  }

}