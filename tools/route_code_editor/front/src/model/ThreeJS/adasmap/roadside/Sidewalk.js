import { SimpleAreaBasedModel, SimpleAreaBasedModelData } from '../common/SimpleAreaBasedModel';
/**
 * @author Taiga Nishiyama
 * 側帯 - 歩道
 * Roadside - Sidewalk
 **/
export class Sidewalk extends SimpleAreaBasedModel{

  constructor(sidewalkData, area){
    super(sidewalkData, area, 'Sidewalk');
    this.area.changeDefaultColor(0xff7b00);
    this.defaultColor = 0xff7b00;
    this.hilight = false;
    this.roadside = null;
  }
  setRoadside(roadside){
    this.roadside = roadside;
  }
  delete(){
    this.parent.remove(this);
  }

}
export class SidewalkData extends SimpleAreaBasedModelData{

  constructor(id, aid, link_id) {
    super(id, aid, link_id);
  }
  //toCsv() and getCsvHeader() are in SimpleAreaBasedModelData
  
}
