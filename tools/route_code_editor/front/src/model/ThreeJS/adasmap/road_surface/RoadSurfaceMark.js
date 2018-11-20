import { SimpleAreaBasedModel } from '../common/SimpleAreaBasedModel';
/**
 * @author Taiga Nishiyama
 * 路面 - 路面マーク
 * Road surface - RoadSurfaceMark
 **/
export class RoadSurfaceMark extends SimpleAreaBasedModel{

  constructor(road_surface_mark_data, area){
    super(road_surface_mark_data, area, 'RoadSurfaceMark');
    this.area.changeDefaultColor(0xff7b00);
    this.area.changeDefaultPointSize(0.02);
    this.defaultColor = 0xff7b00;
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
export class RoadSurfaceMarkData {

  constructor(id, aid, type, link_id) {
    this.id = parseInt(id, 10);
    this.aid = parseInt(aid, 10);
    this.type = type;
    this.link_id = parseInt(link_id, 10);
  }
  toCsv() {
    return this.id + ',' + this.aid + ',' + this.type + ',' + this.link_id + '\n';
  }
  getCsvHeader(){
    return 'ID,AID,Type,LinkID\n';
  }

}