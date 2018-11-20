import { SimpleAreaBasedModel, SimpleAreaBasedModelData } from '../common/SimpleAreaBasedModel';
/**
 * @author Taiga Nishiyama
 * 道路形状 - 交差点
 * Road shape - Intersection
 **/
export class Intersection extends SimpleAreaBasedModel{

  constructor(intersectionData, area){
    super(intersectionData, area, 'Intersection');
    this.area.changeDefaultColor(0xff7b00);
    this.defaultColor = 0xff7b00;
    this.hilight = false;
    this.road_shape = null;
  }
  setRoadShape(road_shape){
    this.road_shape = road_shape;
  }
  delete(){
    this.parent.remove(this);
  }

}
export class IntersectionData extends SimpleAreaBasedModelData{

  constructor(id, aid, link_id) {
    super(id, aid, link_id);
  }
  //toCsv() and getCsvHeader() are in SimpleAreaBasedModelData
  
}
