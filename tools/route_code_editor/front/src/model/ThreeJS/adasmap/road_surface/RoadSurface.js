import { Whiteline, WhitelineData, Stopline, StoplineData, 
  Zebrazone, ZebrazoneData, Crosswalk, CrosswalkData,
  RoadSurfaceMark, RoadSurfaceMarkData } from './index';
import ModelCreateHelper from '../../../../util/ModelCreateHelper';

export default class RoadSurface extends THREE.Group {

  constructor() {
    super();
    this.name = 'RoadSurface';
    this.max_id = {};
        
    // Whitepoint
    this.whitelines = new THREE.Group();
    this.whitelines.getSelectableObjects = () => {     
      return this.whitelines.children.map(whiteline => whiteline.line);
    };
    this.whiteline_points = new THREE.Group();
    this.whiteline_points.getSelectableObjects = function () { return this.children; };
    this.add(this.whitelines);
    this.add(this.whiteline_points);
    // Stopline
    this.stoplines = new THREE.Group();
    this.stoplines.getSelectableObjects = () => {
      return this.stoplines.children.map(stopline => stopline.line);
    };
    this.stopline_points = new THREE.Group();
    this.stopline_points.getSelectableObjects = function () { return this.children; };
    this.add(this.stoplines);
    this.add(this.stopline_points);
    // Zebrazone
    this.zebrazones = new THREE.Group();
    this.zebrazones.getSelectableObjects = () => {
      let objects = [];
      this.zebrazones.children.forEach(zebrazone => {
        objects = objects.concat(zebrazone.area.getSelectableObjects());
      });
      return objects;
    };
    this.add(this.zebrazones);
    // Crosswalk
    this.crosswalks = new THREE.Group();
    this.crosswalks.getSelectableObjects = () => {
      let objects = [];
      this.crosswalks.children.forEach(crosswalk => {
        objects = objects.concat(crosswalk.area.getSelectableObjects());
      });
      return objects;
    };
    this.add(this.crosswalks);
    // RoadSurfaceMark
    this.road_surface_marks = new THREE.Group();
    this.road_surface_marks.getSelectableObjects = () => {
      let objects = [];
      this.road_surface_marks.children.forEach(road_surface_mark => {
        objects = objects.concat(road_surface_mark.area.getSelectableObjects());
      });
      return objects;
    };
    this.add(this.road_surface_marks);
  }
  /**
   * @param {object} models
   * {
   *   points: array of point
   *   lines: array of line,
   * }
   * @param {object} data
   * {
   *   point_data_list: array of point_data,
   *   line_data_list: array of line_data,
   *   whiteline_data_list: array of whiteline_data,
   *   stopline_data_list: array of stopline_data,
   *   ,,,
   * }
   */
  static load(models, data) {
    let road_surface = new RoadSurface();
    // Whiteline
    if (models.lines && data.whiteline_data_list) {
      road_surface.loadLineBasedObject(models, data, 'whiteline', Whiteline);
    }
    // Stopline
    if (models.lines && data.stopline_data_list) {
      road_surface.loadLineBasedObject(models, data, 'stopline', Stopline);
    }
    // Zebrazone
    if (models.areas && data.zebrazone_data_list) {
      road_surface.loadAreaBasedObject(models, data, 'zebrazone', Zebrazone);
    }
    // Crosswalk
    if (models.areas && data.crosswalk_data_list) {
      road_surface.loadAreaBasedObject(models, data, 'crosswalk', Crosswalk);
    }
    // RoadSurfaceMark
    if (models.areas && data.road_surface_mark_data_list) {
      road_surface.loadAreaBasedObject(models, data, 'road_surface_mark', RoadSurfaceMark);
    }
    return road_surface;
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

  createLineBasedObjectFromPosition(DataClass, position){
    let line = ModelCreateHelper.createLineFromPosition(this.parent, position);
    if(DataClass === Whiteline){
      let id = this.incrementMaxId('whiteline');
      let whiteline_data = new WhitelineData(id, line.data.lid, 0, 'W', 0, 0);
      let whiteline= new Whiteline(whiteline_data, line);
      this.addLineBasedObject(whiteline, 'whiteline');
      return whiteline;
    }else if(DataClass === Stopline){
      let id = this.incrementMaxId('stopline');
      let stopline_data = new StoplineData(id, line.data.lid, 0, 0, 0);
      let stopline = new Stopline(stopline_data, line);
      this.addLineBasedObject(stopline, 'stopline');
      return stopline;
    }
  }
  createAreaBasedObjectFromPosition(ModelClass, position){
    let area = ModelCreateHelper.createAreaFromPosition(this.parent, position);
    if(ModelClass === Zebrazone){
      let id = this.incrementMaxId('zebrazone');
      let data = new ZebrazoneData(id, area.data.aid, 0);
      let zebrazone = new Zebrazone(data, area);
      this.addObject(zebrazone, 'zebrazone');
      return zebrazone;
    }else if(ModelClass === Crosswalk){
      let id = this.incrementMaxId('crosswalk');
      let data = new CrosswalkData(id, area.data.aid, 1, 0, 0);
      let crosswalk = new Crosswalk(data, area);
      this.addObject(crosswalk, 'crosswalk');
      return crosswalk;
    }else if(ModelClass === RoadSurfaceMark){
      let id = this.incrementMaxId('road_surface_mark');
      let data = new RoadSurfaceMarkData(id, area.data.aid, '', 0);
      let road_surface_mark = new RoadSurfaceMark(data, area);
      this.addObject(road_surface_mark, 'road_surface_mark');
      return road_surface_mark;
    }
  }
  // lineBasedObjectを追加する
  /**
   * @param {Curb or Roadedge} lineBasedObject - 前または後のlineBasedOjbect
   * @param {Point} point - 新しく追加するlineBasedObjectの一方の端
   */
  createLineBasedObjectAlongWithExisting(lineBasedObject, point){
    let { line, point: new_point } = ModelCreateHelper.createLineAlongWithExisting(this.parent, lineBasedObject.line, point);
    if(lineBasedObject instanceof Whiteline){
      let id = this.incrementMaxId('whiteline');
      let whiteline_data = new WhitelineData(id, line.data.lid, 0, 'W', 0, 0);
      let whiteline = new Whiteline(whiteline_data, line);
      this.addLineBasedObject(whiteline, 'whiteline');
      return { lineBasedObject: whiteline, point: new_point };
    }else if(lineBasedObject instanceof Stopline){
      let id = this.incrementMaxId('stopline');
      let stopline_data = new StoplineData(id, line.data.lid, 0, 0, 0);
      let stopline = new Stopline(stopline_data, line);
      this.addLineBasedObject(stopline, 'stopline');
      return { lineBasedObject: stopline, point: new_point };
    }
  }

  addObject(object, name){
    if(this.max_id[name] === undefined){
      this.max_id[name] = 0;
    }
    if(object.data.id > this.max_id[name]){
      this.max_id[name] = object.data.id;
    }
    object.setRoadSurface(this);
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
  incrementMaxId(name) {
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
    return [this.whitelines, this.whiteline_points, this.stoplines, this.stopline_points, this.zebrazones, this.crosswalks, this.road_surface_marks];
  }

}
