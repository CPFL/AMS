import { Area } from './Area';

// Three.js
import * as THREE from 'three';

/**
 * @author Taiga Nishiyama
 * 
 **/
export class Line extends THREE.Line {

  constructor(lineData, before_point, foward_point) {
    let material = new THREE.LineBasicMaterial({ color: 0xffffff });
    let geometry = new THREE.Geometry();
    geometry.vertices.push(before_point.position, foward_point.position);
    super(geometry, material);
    this.defaultColor = 0xffffff;
    this.data = lineData;
    before_point.addUpdateObject(this);
    this.before_point = before_point;
    foward_point.addUpdateObject(this);
    this.foward_point = foward_point;
    this.before_line = null;
    this.foward_line = null;
    this.hilight = false;
    this.updateObjects = [];
    this.dashed = false;
  }
  initializeRelatedLines(before_line, foward_line){
    if(before_line){
      this.before_line = before_line;
    }
    if(foward_line){
      this.foward_line = foward_line;
    }
  }
  addUpdateObject(object){
    this.updateObjects.push(object);
  }

  setBeforeLine(before_line){
    this.before_line = before_line;
    this.before_line.foward_line = this;
    this.before_line.updateRelatedObjects();
    this.updateRelatedObjects();
  }
  setFowardLine(foward_line){
    this.foward_line = foward_line;
    this.foward_line.before_line = this;
    this.foward_line.updateRelatedObjects();
    this.updateRelatedObjects();
  }
  resetBeforeLine(){
    if(this.before_line != null){
      this.before_line.foward_line = null;
      this.before_line.updateRelatedObjects();
      this.before_line = null;
      this.updateRelatedObjects();
    }else{
      console.error('before_line is null');
    }
  }
  resetFowardLine(){
    if(this.foward_line != null){
      // foward_lineのbefore_line(つまり自分)もnullにする
      this.foward_line.before_line = null;
      this.foward_line.updateRelatedObjects();
      this.foward_line = null;
      this.updateRelatedObjects();
    }else{
      console.error('foward_line is null');
    }
  }
  // Call this before delete line object
  removeLineReferences(){
    this.resetBeforeLine();
    this.resetFowardLine();
  }
  removePointReferences(){
    this.before_point.removeUpdateObject(this);
    this.foward_point.removeUpdateObject(this);
  }
  enableDash(){
    this.dashed = true;
    this.updateMaterial();
  }
  disableDash(){
    this.dashed = false;
    this.updateMaterial();
  }
  /**
   * Change Foward Point
   * @param {Point} new_point 
   */
  changeFowardPoint(new_point) {
    // 古いpointを除去
    let index = this.geometry.vertices.indexOf(this.foward_point.position);
    this.geometry.vertices.splice(index, 1);
    this.geometry.vertices.push(new_point.position);
    this.foward_point.removeUpdateObject(this);
    // 新しいpointを設定
    this.foward_point = new_point;
    this.foward_point.addUpdateObject(this);
    this.data.fpid = new_point.data.pid;
  }
  /**
   * Change Before Point
   * @param {Point} new_point 
   */
  changeBeforePoint(new_point) {
    // 古いpointを除去
    let index = this.geometry.vertices.indexOf(this.before_point.position);
    this.geometry.vertices.splice(index, 1);
    this.geometry.vertices.push(new_point.position);
    this.before_point.removeUpdateObject(this);
    // 新しいpointを設定
    this.before_point = new_point;
    this.before_point.addUpdateObject(this);
    this.data.bpid = new_point.data.pid;
  }

  /**********************************
   * Object3D model interface methds *
   **********************************/
  updateRelatedObjects(updatedObjects = []) {
    this.geometry.verticesNeedUpdate = true;
    this.geometry.boundingSphere = null;
    if(this.before_line){
      this.data.blid = this.before_line.data.lid;
    }else{
      this.data.blid = 0;
    }
    if(this.foward_line){
      this.data.flid = this.foward_line.data.lid;
    }else{
      this.data.flid = 0;
    }
    this.updateObjects.forEach(obj => obj.updateRelatedObjects(updatedObjects.push(this)));
    this.computeLineDistances();
  }
  enableHighlight(color) {
    this.hilight = true;
    this.updateMaterial(color);
    this.updateObjects.forEach(obj => {
      if(obj instanceof Area){
        obj.enableHighlight();
      }
    });
  }
  disableHighlight() {
    this.hilight = false;
    this.updateMaterial();
    this.updateObjects.forEach(obj => {
      if(obj instanceof Area){
        obj.disableHighlight();
      }
    });
  }
  updateMaterial(color) {
    let material_color;
    if(this.hilight){
      material_color = color || 0xf6ff4c;
    }else{
      material_color = this.defaultColor;
    }
    let material;
    if(this.dashed){
      material = new THREE.LineDashedMaterial({ color: material_color, dashSize: 0.05, gapSize: 0.05 });
      this.computeLineDistances();
    }else{
      material = new THREE.LineBasicMaterial({ color: material_color });
    }
    this.material = material;
  }
  changeDefaultColor(color) {
    this.defaultColor = color;
    this.updateMaterial();
  }

}

export class LineData {

  constructor(lid, bpid, fpid, blid, flid){
    this.lid = parseInt(lid, 10);
    this.bpid = parseInt(bpid, 10);
    this.fpid = parseInt(fpid, 10);
    this.blid = parseInt(blid, 10);
    this.flid = parseInt(flid, 10);
  }
  toCsv() {
    return this.lid + ',' + this.bpid + ',' + this.fpid + ','
           + this.blid + ',' + this.flid + '\n';
  }
  getCsvHeader(){
    return 'LID,BPID,FPID,BLID,FLID\n';
  }

}
