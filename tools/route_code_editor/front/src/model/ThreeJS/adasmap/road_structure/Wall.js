import { SimpleAreaBasedModel, SimpleAreaBasedModelData } from '../common/SimpleAreaBasedModel';
/**
 * @author Taiga Nishiyama
 **/
export class Wall extends SimpleAreaBasedModel{

  constructor(wallData, area){
    super(wallData, area, 'Wall');
    this.area.changeDefaultColor(0xd0ff6d);
    this.defaultColor = 0xd0ff6d;
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
export class WallData extends SimpleAreaBasedModelData{

  constructor(id, aid, link_id) {
    super(id, aid, link_id);
  }
  //toCsv() and getCsvHeader() are in SimpleAreaBasedModelData

}
