import { SimpleAreaBasedModel, SimpleAreaBasedModelData } from '../common/SimpleAreaBasedModel';
/**
 * @author Taiga Nishiyama
 **/
export class Fence extends SimpleAreaBasedModel{

  constructor(fenceData, area){
    super(fenceData, area, 'Fence');
    this.area.changeDefaultColor(0xff7b00);
    this.defaultColor = 0xff7b00;
    this.hilight = false;
    this.roadside = null;
  }
  setRoadStructure(road_structure){
    this.road_structure = road_structure;
  }
  delete(){
    this.parent.remove(this);
  }

}
export class FenceData extends SimpleAreaBasedModelData{

  constructor(id, aid, link_id) {
    super(id, aid, link_id);
  }
  //toCsv() and getCsvHeader() are in SimpleAreaBasedModelData

}
