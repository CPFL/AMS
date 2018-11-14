import { Curb, CurbData, Roadedge, RoadedgeData,
  Gutter, GutterData, Intersection, IntersectionData } from './index';
import ModelCreateHelper from '../../../../util/ModelCreateHelper';

export default class RoadShape extends THREE.Group {

  constructor() {
    super();
    this.name = 'RoadShape';
    this.max_id = {};
    // Curb
    this.curbs = new THREE.Group();
    this.curbs.getSelectableObjects = () => {
      return this.curbs.children.map(curb => curb.line);
    };
    this.add(this.curbs);
    this.curb_points = new THREE.Group();
    this.curb_points.getSelectableObjects = () => {
      return this.curb_points.children;
    };
    this.add(this.curb_points);
    // Roadedge
    this.roadedges = new THREE.Group();
    this.roadedges.getSelectableObjects = () => {     
      return this.roadedges.children.map(roadedge => roadedge.line);
    };
    this.add(this.roadedges);
    this.roadedge_points = new THREE.Group();
    this.roadedge_points.getSelectableObjects = function () { return this.children; };
    this.add(this.roadedge_points);
    // Gutter
    this.gutters = new THREE.Group();
    this.gutters.getSelectableObjects = () => {
      let objects = [];
      this.gutters.children.forEach(gutter => {
        objects = objects.concat(gutter.area.getSelectableObjects());
      });
      return objects;
    };
    this.add(this.gutters);
    // Intersection
    this.intersections = new THREE.Group();
    this.intersections.getSelectableObjects = () => {
      let objects = [];
      this.intersections.children.forEach(intersection => {
        objects = objects.concat(intersection.area.getSelectableObjects());
      });
      return objects;
    };
    this.add(this.intersections);
  }
  /**
   * @param {object} models
   * {
   *   points: array of point
   *   lines: array of line,
   * }
   * @param {object} data
   * {
   *   curb_data_list: array of curb_data,
   *   ,,,
   * }
   */
  static load(models, data) {
    let road_shape = new RoadShape();
    // Curb
    if (models.lines && data.curb_data_list) {
      road_shape.loadLineBasedObject(models, data, 'curb', Curb);
    }
    // Roadedge
    if (models.lines && data.roadedge_data_list) {
      road_shape.loadLineBasedObject(models, data, 'roadedge', Roadedge);
    }
    // Gutter
    if (models.areas && data.gutter_data_list) {
      road_shape.loadAreaBasedObject(models, data, 'gutter', Gutter);
    }
    // Intersection
    if (models.areas && data.intersection_data_list) {
      road_shape.loadAreaBasedObject(models, data, 'intersection', Intersection);
    }
    
    return road_shape;
  }

  createLineBasedObjectFromPosition(ModelClass, position){
    let line = ModelCreateHelper.createLineFromPosition(this.parent, position);
    if(ModelClass ===  Curb){
      let curb_id = this.incrementMaxId('curb');
      let curb_data = new CurbData(curb_id, line.data.lid, 0, 0, 0, 0);
      let curb = new Curb(curb_data, line);
      this.addLineBasedObject(curb, 'curb');
      return curb;
    }else if(ModelClass === Roadedge){
      let roadedge_id = this.incrementMaxId('roadedge');
      let roadedge_data = new RoadedgeData(roadedge_id, line.data.lid, 0);
      let roadedge = new Roadedge(roadedge_data, line);
      this.addLineBasedObject(roadedge, 'roadedge');
      return roadedge;
    }
  }
  createAreaBasedObjectFromPosition(ModelClass, position){
    let area = ModelCreateHelper.createAreaFromPosition(this.parent, position);
    if(ModelClass === Gutter){
      let id = this.incrementMaxId('gutter');
      let data = new GutterData(id, area.data.aid, 1, 0);
      let gutter = new Gutter(data, area);
      this.addObject(gutter, 'gutter');
      return gutter;
    }else if(ModelClass === Intersection){
      let id = this.incrementMaxId('intersection');
      let data = new IntersectionData(id, area.data.aid, 0);
      let intersection = new Intersection(data, area);
      this.addObject(intersection, 'intersection');
      return intersection;
    }
  }
  // lineBasedObjectを追加する
  /**
   * @param {Curb or Roadedge} lineBasedObject - 前または後のlineBasedOjbect
   * @param {*} point - 新しく追加するlineBasedObjectの一方の端
   */
  createLineBasedObjectAlongWithExisting(lineBasedObject, point){
    let { line, point: new_point } = ModelCreateHelper.createLineAlongWithExisting(this.parent, lineBasedObject.line, point);
    if(lineBasedObject instanceof Curb){
      let curb_id = this.incrementMaxId('curb');
      let curb_data = new CurbData(curb_id, line.data.lid, 0, 0, 0, 0);
      let curb = new Curb(curb_data, line);
      this.addLineBasedObject(curb, 'curb');
      return { lineBasedObject: curb, point: new_point };
    }else if(lineBasedObject instanceof Roadedge){
      let roadedge_id = this.incrementMaxId('roadedge');
      let roadedge_data = new RoadedgeData(roadedge_id, line.data.lid, 0);
      let roadedge = new Roadedge(roadedge_data, line);
      this.addLineBasedObject(roadedge, 'roadedge');
      return { lineBasedObject: roadedge, point: new_point };
    }
  }

  loadLineBasedObject(models, data, name, ObjectClass){
    data[name + '_data_list'].forEach((obj_data)=>{
      let line = models.lines.find(line => line.data.lid === obj_data.lid);
      let obj = new ObjectClass(obj_data, line);
      this.addLineBasedObject(obj, name);
    });
  }

  loadAreaBasedObject(models, data, name, ObjectClass){
    data[name + '_data_list'].forEach((obj_data) => {
      let area = models.areas.find(area => area.data.aid === obj_data.aid);
      let obj = new ObjectClass(obj_data, area);
      this.addObject(obj, name);
    });
  }

  addObject(object, name){
    if(this.max_id[name] === undefined){
      this.max_id[name] = 0;
    }
    if(object.data.id > this.max_id[name]){
      this.max_id[name] = object.data.id;
    }
    object.setRoadShape(this);
    this[name+'s'].add(object);
  }
  addLineBasedObject(object, name){
    this.addObject(object, name);
    let line = object.line;
    if(!this[name + '_points'].children.find(p => p === line.before_point)){
      this[name + '_points'].add(line.before_point);
    }
    if(!this[name + '_points'].children.find(p => p === line.foward_point)){
      this[name + '_points'].add(line.foward_point);
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

  removeLineBasedObject(lineBasedObject){
    // PointによるLineへの参照を除去
    lineBasedObject.line.removePointReferences();
    // BeforeLine, FowardLineによるLineへの参照を除去
    lineBasedObject.line.removeLineReferences();
    // Pointが誰からも参照されていない時除去
    if(lineBasedObject.line.before_point.updateObjects.length === 0){
      this.removeLineBasedObjectPoint(lineBasedObject.line.before_point);
    }
    if(lineBasedObject.line.foward_point.updateObjects.length === 0){
      this.removeLineBasedObjectPoint(lineBasedObject.line.foward_point);
    }
    // LineBasedObjectを親グループから除去
    let parent = lineBasedObject.parent;
    parent.remove(lineBasedObject);
  }
  removeLineBasedObjectPoint(point){
    let parent = point.parent;
    parent.remove(point);
  }
  getRoadObjectGroups(){
    return [this.curbs, this.curb_points, this.roadedges, this.roadedge_points, this.gutters, this.intersections];
  }

}
