/**
 * @author Taiga Nishiyama
 * Areaを内包するモデル
 **/
export class SimpleAreaBasedModel extends THREE.Group{

  constructor(model_data, area, name){
    super();
    this.name = name;
    this.data = model_data;
    this.area = area;
    this.add(area);
  }

}
export class SimpleAreaBasedModelData {

  constructor(id, aid, link_id) {
    this.id = parseInt(id, 10);
    this.aid = parseInt(aid, 10);
    this.link_id = parseInt(link_id, 10);
  }
  toCsv() {
    return this.id + ',' + this.aid + ',' + this.link_id + '\n';
  }
  getCsvHeader(){
    return 'ID,AID,LinkID\n';
  }

}