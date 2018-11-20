import { SimpleAreaBasedModel } from '../common/SimpleAreaBasedModel';
/**
 * @author Taiga Nishiyama
 * 側帯 - ガードレール
 * Roadside - Guardrail
 **/
export class Guardrail extends SimpleAreaBasedModel{

  constructor(guardrailData, area){
    super(guardrailData, area, 'Guardrail');
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
export class GuardrailData {

  constructor(id, aid, type, link_id) {
    this.id = parseInt(id, 10);
    this.type = parseInt(type, 10);
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
