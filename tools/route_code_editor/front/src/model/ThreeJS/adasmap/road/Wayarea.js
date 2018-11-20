import { SimpleAreaBasedModel } from '../common/SimpleAreaBasedModel';
/**
 * @author Taiga Nishiyama
 * 道路データ - 走行エリア
 * Road data - Way Area
 **/
export class Wayarea extends SimpleAreaBasedModel{

  constructor(wayareaData, area){
    super(wayareaData, area, 'Wayarea');
    this.area.changeDefaultColor(0x00e591);
    this.defaultColor = 0x00e591;
    this.hilight = false;
    this.road = null;
  }
  setRoad(road){
    this.road = road;
  }
  delete(){
    this.parent.remove(this);
  }

}
export class WayareaData {

  constructor(waid, aid) {
    this.waid = parseInt(waid, 10);
    this.aid = parseInt(aid, 10);
  }
  toCsv(){
    return this.waid + ',' + this.aid + '\n';
  }
  getCsvHeader(){
    return 'WAID,AID\n';
  }

}
