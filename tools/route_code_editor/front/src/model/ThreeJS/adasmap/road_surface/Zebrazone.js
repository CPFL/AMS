import { SimpleAreaBasedModel, SimpleAreaBasedModelData } from '../common/SimpleAreaBasedModel';/**
 * @author Taiga Nishiyama
 * 路面 - ゼブラゾーン
 * Road surface - Zebrazone
 **/
export class Zebrazone extends SimpleAreaBasedModel{

  constructor(zebrazoneData, area){
    super(zebrazoneData, area, 'Zebrazone');
    this.area.changeDefaultColor(0xff7b00);
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
export class ZebrazoneData extends SimpleAreaBasedModelData{

  constructor(id, aid, link_id) {
    super(id, aid, link_id);
  }
  //toCsv() and getCsvHeader() are in SimpleAreaBasedModelData

}
