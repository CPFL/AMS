import { Sidewalk, SidewalkData, 
  Guardrail, GuardrailData } from './index';
import ModelCreateHelper from '../../../../util/ModelCreateHelper';

export default class Roadside extends THREE.Group {

  constructor() {
    super();
    this.name = 'Roadside';
    this.max_id = {};
    // Guardrail
    this.guardrails = new THREE.Group();
    this.guardrails.getSelectableObjects = () => {
      let objects = [];
      this.guardrails.children.forEach(guardrail => {
        objects = objects.concat(guardrail.area.getSelectableObjects());
      });
      return objects;
    };
    this.add(this.guardrails);
    // Sidewalk
    this.sidewalks = new THREE.Group();
    this.sidewalks.getSelectableObjects = () => {
      let objects = [];
      this.sidewalks.children.forEach(sidewalk => {
        objects = objects.concat(sidewalk.area.getSelectableObjects());
      });
      return objects;
    };
    this.add(this.sidewalks);
  }
  /**
   * @param {object} models
   * {
   *   areas: array of area
   * }
   * @param {object} data
   * {
   *   gutter_data_list: array of guardrail_data,
   *   intersection_data_list:  array of intersection_data,
   * }
   */
  static load(models, data) {
    let roadside = new Roadside();
    // Gutter
    if (models.areas && data.gutter_data_list) {
      roadside.loadAreaBasedObject(models, data, 'guardrail', Guardrail);
    }
    // Intersection
    if (models.areas && data.intersection_data_list) {
      roadside.loadAreaBasedObject(models, data, 'sidewalk', Sidewalk);
    }
    return roadside;
  }
  loadAreaBasedObject(models, data, name, ObjectClass){
    data[name + '_data_list'].forEach((obj_data) => {
      let area = models.areas.find(area => area.data.aid === obj_data.aid);
      let obj = new ObjectClass(obj_data, area);
      this.addObject(obj, name);
    });
  }
  
  createAreaBasedObjectFromPosition(ModelClass, position){
    let area = ModelCreateHelper.createAreaFromPosition(this.parent, position);
    if(ModelClass === Guardrail){
      let id = this.incrementMaxId('guardrail');
      let data = new GuardrailData(id, area.data.aid, 1, 0);
      let guardrail = new Guardrail(data, area);
      this.addObject(guardrail, 'guardrail');
      return guardrail;
    }else if(ModelClass === Sidewalk){
      let id = this.incrementMaxId('sidewalk');
      let data = new SidewalkData(id, area.data.aid, 0);
      let sidewalk = new Sidewalk(data, area);
      this.addObject(sidewalk, 'sidewalk');
      return sidewalk;
    }
  }
  // max_id.name を1増加し、その値を返す
  incrementMaxId(name){
    if(this.max_id[name] === undefined){
      this.max_id[name] = 0;
    }
    this.max_id[name] += 1;
    return this.max_id[name];
  }
  addObject(object, name){
    if(this.max_id[name] === undefined){
      this.max_id[name] = 0;
    }
    if(object.data.id > this.max_id[name]){
      this.max_id[name] = object.data.id;
    }
    object.setRoadside(this);
    this[name+'s'].add(object);
  }
  getRoadObjectGroups(){
    return [this.guardrails, this.sidewalks];
  }

}
