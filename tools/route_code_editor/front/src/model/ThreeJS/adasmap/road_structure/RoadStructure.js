import { Roadsign, RoadsignData, Poledata, PoledataData,
  Utilitypole, UtilitypoleData, Signaldata, SignaldataData,
  Curvemirror, CurvemirrorData, Streetlight, StreetlightData,
  Wall, WallData, Fence, FenceData, RailroadCrossing, RailroadCrossingData } from './index';
import ModelCreateHelper from '../../../../util/ModelCreateHelper';

export default class RoadStructure extends THREE.Group {

  constructor() {
    super();
    this.name = 'RoadStructure';
    this.max_id = {};

    // Roadsign(標識)
    this.roadsigns = new THREE.Group();
    this.roadsigns.getSelectableObjects = () => {
      let objects = [];
      this.roadsigns.children.forEach(roadsign => {
        objects = objects.concat(roadsign.getSelectableObjects());
      });
      return objects;
    };
    this.add(this.roadsigns);
    // Poledata(ポール)
    this.poledatas = new THREE.Group();
    this.poledatas.getSelectableObjects = () => {
      return this.poledatas.children.map(poledata => poledata.pole);
    };
    this.add(this.poledatas);
    // Utilitypole
    this.utilitypoles = new THREE.Group();
    this.utilitypoles.getSelectableObjects = () => {
      return this.utilitypoles.children.map(utilitypole => utilitypole.pole);
    };
    this.add(this.utilitypoles);
    // Signaldata
    this.signaldatas = new THREE.Group();
    this.signaldatas.getSelectableObjects = () => {
      return this.signaldatas.children.map(signaldata => signaldata.vector);
    };
    this.add(this.signaldatas);
    // Curvemirror
    this.curvemirrors = new THREE.Group();
    this.curvemirrors.getSelectableObjects = () => {
      return this.curvemirrors.children.map(curvemirror => curvemirror.vector);
    };
    this.add(this.curvemirrors);
    // Streetlights
    this.streetlights = new THREE.Group();
    this.streetlights.getSelectableObjects = () => {
      let objects = [];
      this.streetlights.children.forEach(streetlight => { 
        objects = objects.concat(streetlight.children);
      });
      return objects;
    };
    this.add(this.streetlights);

    // Wall
    this.walls = new THREE.Group();
    this.walls.getSelectableObjects = () => {
      let objects = [];
      this.walls.children.forEach(wall => {
        objects = objects.concat(wall.area.getSelectableObjects());
      });
      return objects;
    };
    this.add(this.walls);
    // Fence
    this.fences = new THREE.Group();
    this.fences.getSelectableObjects = () => {
      let objects = [];
      this.fences.children.forEach(fence => {
        objects = objects.concat(fence.area.getSelectableObjects());
      });
      return objects;
    };
    this.add(this.fences);
    // RailroadCrossing
    this.railroad_crossings = new THREE.Group();
    this.railroad_crossings.getSelectableObjects = () => {
      let objects = [];
      this.railroad_crossings.children.forEach(railroad_crossing => {
        objects = objects.concat(railroad_crossing.area.getSelectableObjects());
      });
      return objects;
    };
    this.add(this.railroad_crossings);
  }
  /**
   * @param {object} models
   * {
   *   points: array of point
   *   lines: array of line,
   *   vectors: array of vector,
   *   ...
   * }
   * @param {object} data
   * {
   *   roadsign_data_list: array of roadsign_data,
   *   ...
   * }
   */
  static load(models, data) {
    let road_structure = new RoadStructure();
    // Roadsign - 標識
    if (models.vectors && models.poles && data.roadsign_data_list) {
      road_structure.loadVectorBasedObject(models, data, 'roadsign', Roadsign);
    }
    // Poledata - ポール
    if (models.poles && data.poledata_data_list){
      road_structure.loadPoleBasedObject(models, data, 'poledata', Poledata);
    }
    // Utilitypole - ポール
    if (models.poles && data.utilitypole_data_list) {
      road_structure.loadPoleBasedObject(models, data, 'utilitypole', Utilitypole);
    }
    // Signaldata - 信号
    if (models.vectors && models.poles && data.signaldata_data_list) {
      road_structure.loadVectorBasedObject(models, data, 'signaldata', Signaldata);
    }
    // Curvemirror - カーブミラー
    if (models.vectors && models.poles && data.curvemirror_data_list) {
      road_structure.loadVectorBasedObject(models, data, 'curvemirror', Curvemirror);
    }
    // Streetlight - 街灯
    if (models.lines && data.streetlight_data_list) { 
      data.streetlight_data_list.forEach(streetlight_data => {
        let line = models.lines.find(line => line.data.lid === streetlight_data.lid);
        let pole = models.poles.find(pole => pole.data.plid === streetlight_data.plid);
        let streetlight = new Streetlight(streetlight_data, line);
        streetlight.setPole(pole);
        road_structure.addObject(streetlight, 'streetlight');
      });
    }
    // Wall
    if (models.areas && data.wall_data_list) {
      road_structure.loadAreaBasedObject(models, data, 'wall', Wall);
    }
    // Fence
    if (models.areas && data.fence_data_list) {
      road_structure.loadAreaBasedObject(models, data, 'fence', Fence);
    }
    // RailroadCrossing
    if (models.areas && data.railroad_crossing_data_list) {
      road_structure.loadAreaBasedObject(models, data, 'railroad_crossing', RailroadCrossing);
    }
    return road_structure;
  }

  createStreetlight(position){
    let line = ModelCreateHelper.createLineFromPosition(this.parent, position);
    let id = this.incrementMaxId('streetlight');
    let streetlight_data = new StreetlightData(id, line.data.lid, 0, 0);
    let streetlight = new Streetlight(streetlight_data, line);
    this.addObject(streetlight, 'streetlight');
    return streetlight;
  }

  createAreaBasedObjectFromPosition(ModelClass, position){
    let area = ModelCreateHelper.createAreaFromPosition(this.parent, position);
    if(ModelClass === Wall){
      let id = this.incrementMaxId('wall');
      let data = new WallData(id, area.data.aid, 0);
      let wall = new Wall(data, area);
      this.addObject(wall, 'wall');
      return wall;
    }else if(ModelClass === Fence){
      let id = this.incrementMaxId('fence');
      let data = new FenceData(id, area.data.aid, 0);
      let fence = new Fence(data, area);
      this.addObject(fence, 'fence');
      return fence;
    }else if(ModelClass === RailroadCrossing){
      let id = this.incrementMaxId('railroad_crossing');
      let data = new RailroadCrossingData(id, area.data.aid, 0);
      let railroad_crossing = new RailroadCrossing(data, area);
      this.addObject(railroad_crossing, 'railroad_crossing');
      return railroad_crossing;
    }
  }
  createVectorBasedObjectFromPosition(ModelClass, position){
    // point生成
    // vector生成
    let vector = ModelCreateHelper.createVectorFromPosition(this.parent, position, 0, 90);
    if(ModelClass === Roadsign){
      let id = this.incrementMaxId('roadsign');
      let roadsign_data = new RoadsignData(id, vector.data.vid, 0, '', 0);
      let roadsign = new Roadsign(roadsign_data, vector);
      this.addObject(roadsign, 'roadsign');
      return roadsign;
    }else if(ModelClass === Signaldata){
      let id = this.incrementMaxId('signaldata');
      let signaldata_data = new SignaldataData(id, vector.data.vid, 0, 1, 0);
      let signaldata = new Signaldata(signaldata_data, vector);
      this.addObject(signaldata, 'signaldata');
      return signaldata;
    }else if(ModelClass === Curvemirror){
      let id = this.incrementMaxId('curvemirror');
      let curvemirror_data = new CurvemirrorData(id, vector.data.vid, 0, 0, 0);
      let curvemirror = new Curvemirror(curvemirror_data, vector);
      this.addObject(curvemirror, 'curvemirror');
      return curvemirror;
    }
  }
  createPoleBasedObjectFromPosition(ModelClass, position){
    let pole = ModelCreateHelper.createPoleFromPosition(this.parent, position);
    if(ModelClass === Poledata){
      let id = this.incrementMaxId('poledata');
      let poledata_data = new PoledataData(id, pole.data.plid, 0);
      let poledata = new Poledata(poledata_data, pole);
      this.addObject(poledata, 'poledata');
      return poledata;
    }else if(ModelClass === Utilitypole){
      let id = this.incrementMaxId('utilitypole');
      let utilitypole_data = new UtilitypoleData(id, pole.data.plid, 0);
      let utilitypole = new Utilitypole(utilitypole_data, pole);
      this.addObject(utilitypole, 'utilitypole');
      return utilitypole;
    }
  }
  cloneVectorBasedObject(vectorBasedObject){
    let vector = ModelCreateHelper.cloneVector(this.parent, vectorBasedObject.vector);
    if(vectorBasedObject instanceof Roadsign){
      let id = this.incrementMaxId('roadsign');
      let roadsign_data = new RoadsignData(id, vector.data.vid, 0, '', 0);
      let roadsign = new Roadsign(roadsign_data, vector);
      this.addObject(roadsign, 'roadsign');
      return roadsign;
    }else if(vectorBasedObject instanceof Signaldata){
      let id = this.incrementMaxId('signaldata');
      let signaldata_data = new SignaldataData(id, vector.data.vid, 0, 1, 0);
      let signaldata = new Signaldata(signaldata_data, vector);
      this.addObject(signaldata, 'signaldata');
      return signaldata;
    }else if(vectorBasedObject instanceof Curvemirror){
      let id = this.incrementMaxId('curvemirror');
      let curvemirror_data = new CurvemirrorData(id, vector.data.vid, 0, 0, 0);
      let curvemirror = new Curvemirror(curvemirror_data, vector);
      this.addObject(curvemirror, 'curvemirror');
      return curvemirror;
    }
  }
  clonePoleBasedObject(poleBasedObject){
    let pole = ModelCreateHelper.clonePole(this.parent, poleBasedObject.pole);
    if(poleBasedObject instanceof Poledata){
      let id = this.incrementMaxId('poledata');
      let poledata_data = new PoledataData(id, pole.data.plid, 0);
      let poledata = new Poledata(poledata_data, pole);
      this.addObject(poledata, 'poledata');
      return poledata;
    }else if(poleBasedObject instanceof Utilitypole){
      let id = this.incrementMaxId('utilitypole');
      let utilitypole_data = new UtilitypoleData(id, pole.data.plid, 0);
      let utilitypole = new Utilitypole(utilitypole_data, pole);
      this.addObject(utilitypole, 'utilitypole');
      return utilitypole;
    }
  }

  // max_id.name を1増加し、その値を返す
  incrementMaxId(name) {
    if (this.max_id[name] === undefined) {
      this.max_id[name] = 0;
    }
    this.max_id[name] += 1;
    return this.max_id[name];
  }

  loadPoleBasedObject(models, data, name, ObjectClass){
    data[name + '_data_list'].forEach(obj_data => {
      let pole = models.poles.find(pole => pole.data.plid === obj_data.plid);
      let obj = new ObjectClass(obj_data, pole);
      this.addObject(obj, name);
    });
  }

  loadVectorBasedObject(models, data, name, ObjectClass){
    data[name + '_data_list'].forEach(obj_data => {
      let vector = models.vectors.find(vector => vector.data.vid === obj_data.vid);
      let pole = models.poles.find(pole => pole.data.plid === obj_data.plid);
      let obj = new ObjectClass(obj_data, vector);
      obj.setPole(pole);
      this.addObject(obj, name);
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
    object.setRoadStructure(this);
    this[name+'s'].add(object);
  }

  getRoadObjectGroups(){
    return [this.roadsigns, this.poledatas, this.utilitypoles, this.signaldatas, this.curvemirrors, this.streetlights, this.walls, this.fences, this.railroad_crossings];
  }

}
