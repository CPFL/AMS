import { SimpleAreaBasedModel, SimpleAreaBasedModelData } from '../common/SimpleAreaBasedModel';
/**
 * @author Taiga Nishiyama
 **/
export class RailroadCrossing extends SimpleAreaBasedModel{

  constructor(railroad_crossing_data, area){
    super(railroad_crossing_data, area, 'RailroadCrossing');
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
export class RailroadCrossingData extends SimpleAreaBasedModelData{

  constructor(id, aid, link_id) {
    super(id, aid, link_id);
  }
  //toCsv() and getCsvHeader() are in SimpleAreaBasedModelData

}
