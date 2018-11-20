import { SimpleAreaBasedModel } from '../common/SimpleAreaBasedModel';
/**
 * @author Taiga Nishiyama
 * 道路形状 - 側溝
 * Road shape - Gutter
 **/
export class Gutter extends SimpleAreaBasedModel{

  constructor(gutterData, area){
    super(gutterData, area, 'Gutter');
    this.area.changeDefaultColor(0x00e591);
    this.defaultColor = 0x00e591;
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
export class GutterData {

  constructor(id, aid, type, link_id) {
    this.id = parseInt(id, 10);
    this.type = parseInt(id, 10);
    this.aid = parseInt(aid, 10);
    this.link_id = parseInt(link_id, 10);
  }

  toCsv() {
    return this.id + ',' + this.aid + ',' + this.type + ',' + this.link_id + '\n';
  }
  getCsvHeader(){
    return 'ID,AID,Type,LinkID\n';
  }

}