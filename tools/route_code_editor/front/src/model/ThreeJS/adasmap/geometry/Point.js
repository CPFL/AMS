import locationUtil from '../../../../util/LocationUtil';
import { Area } from './Area';
/**
 * Basic geometry data
 * Point - 点
 * Define classes Point, PointData, PointDataBuilder
 **/


/**
 * Point
 */
export class Point extends THREE.Mesh {

  /**
   * @param {PointData} pointData
   */
  constructor(pointData) {
    let defaultSize = 0.1;
    super(new THREE.BoxBufferGeometry(defaultSize, defaultSize, defaultSize), new THREE.MeshBasicMaterial({ color: 0xcccccc, opacity: 0.8, transparent: true }));
    this.position.copy(new THREE.Vector3(pointData.ly, pointData.bx, pointData.h));
    this.defaultSize = defaultSize;
    this.data = pointData;
    this.location_util = locationUtil;
    this.updateObjects = [];
    this.defaultColor = 0xcccccc;
    this.hilight = false;
    // PointがVectorの座標として利用されているとき、vectorがセットされる
    // この時、Pointはシーン内では不可視
    // EditPane上での値変更をVectorに移譲する
    this.referringVector = null;
  }

  addUpdateObject(object){
    this.updateObjects.push(object);
  }
  removeUpdateObject(object){
    this.updateObjects = this.updateObjects.filter(obj => obj != object);
  }

  setRefferingVector(vector){
    this.referringVector = vector;
  }

  setBx(bx){
    if(this.referringVector){
      this.referringVector.setBx(bx);
    }else{
      this.position.y = bx;
      this.updateRelatedObjects();
    }
  }
  setLy(ly){
    if(this.referringVector){
      this.referringVector.setLy(ly);
    }else{
      this.position.x = ly;
      this.updateRelatedObjects();
    }
  }
  setH(h){
    if(this.referringVector){
      this.referringVector.setH(h);
    }else{
      this.position.z = h;
      this.updateRelatedObjects();
    }
  }
  

  /**********************************
   * Object3D model interface methds *
   **********************************/
  updateRelatedObjects() {
    // data更新
    this.position.x = Math.round(this.position.x * 1000)/1000;
    this.position.y = Math.round(this.position.y * 1000)/1000;
    this.position.z = Math.round(this.position.z * 1000)/1000;
    this.data.bx = this.position.y;
    this.data.ly = this.position.x;
    this.data.h = this.position.z;
    let { b, l } = this.location_util.toLatLon(this.data.bx, this.data.ly, this.data.ref);
    let { mcode1, mcode2, mcode3 } = this.location_util.toMeshcode(b, l);
    this.data.b = Math.round(this.location_util.toDMS(b) * 10000000)/10000000;
    this.data.l = Math.round(this.location_util.toDMS(l) * 10000000)/10000000;
    this.data.mcode1 = mcode1;
    this.data.mcode2 = mcode2;
    this.data.mcode3 = mcode3;
    // 関連オブジェクト更新
    this.updateObjects.forEach(obj => obj.updateRelatedObjects([this]));
  }
  enableHighlight(color) {
    let material_color = color || 0xf6ff4c;
    this.material = new THREE.MeshBasicMaterial({ color: material_color, opacity: 0.8, transparent: true });
    this.hilight = true;
    this.updateObjects.forEach(obj => {
      if(obj instanceof Area){
        obj.enableHighlight();
      }
    });
  }
  disableHighlight() {
    this.material = new THREE.MeshBasicMaterial({ color: this.defaultColor, opacity: 0.8, transparent: true });
    this.hilight = false;
    this.updateObjects.forEach(obj => {
      if(obj instanceof Area){
        obj.disableHighlight();
      }
    });
  }
  changeDefaultColor(color) {
    this.defaultColor = color;
    if(this.hilight === false){
      this.material = new THREE.MeshBasicMaterial({ color: this.defaultColor, opacity: 0.8, transparent: true });
    }
  }
  changeDefaultSize(size){
    this.defaultSize = size;
    this.geometry = new THREE.BoxBufferGeometry(this.defaultSize, this.defaultSize, this.defaultSize);
  }

}
/**
 * PointData
 * @param {number|string} pid - Point id(int)
 * @param {number|string} b - Latitude 緯度 D.MMSS(JGD2011) (double)
 * @param {number|string} l - Longitude 経度 D.MMSS(JGD2011) (double)
 * @param {number|string} h - Elevation 標高(double)
 * @param {number|string} bx - X cordinate 平面直角座標系X座標(JGD2011) (double)
 * @param {number|string} ly - Y cordinate 平面直角座標系Y座標(JGD2011) (double)
 * @param {number|string} ref - Cordinate system number of Plane Cartesian Cordinate System 平面直角座標系の座標系番号(int)
 * @param {number|string} mcode1 - Primary code 4 digits of the reference region mesh 基準地域メッシュの1次コード4桁 (int)
 * @param {number|string} mcode2 - Secondary code 2 digits of thre reference region mesh 基準地域メッシュの2次コード2桁 (int)
 * @param {number|string} mcode3 - Third code 2 digits of thre reference region mesh 基準地域メッシュの3次コード2桁 (int)
 */
export class PointData {

  constructor(pid, b, l, h, bx, ly, ref, mcode1, mcode2, mcode3) {
    this.pid = parseInt(pid, 10);
    this.b = parseFloat(b);
    this.l = parseFloat(l);
    this.h = parseFloat(h);
    this.bx = parseFloat(bx);
    this.ly = parseFloat(ly);
    this.ref = parseInt(ref);
    this.mcode1 = parseInt(mcode1, 10);
    this.mcode2 = parseInt(mcode2, 10);
    this.mcode3 = parseInt(mcode3, 10);
  }
  toCsv(){
    return this.pid + ',' + this.b + ',' + this.l + ',' + this.h + ','
           + this.bx + ',' + this.ly+ ',' + this.ref+ ','
           + this.mcode1+ ',' + this.mcode2  + ',' + this.mcode3 + '\n';
  }
  getCsvHeader(){
    return 'PID,B,L,H,Bx,Ly,ReF,MCODE1,MCODE2,MCODE3\n';
  }

}
/**
 * PointDataBuilder
 * @example
 * let pointData = new PointDataBuilder().setBx(bx).setLy(ly).setH(h).build();
 */
export class PointDataBuilder {

  constructor() {
    this.pid = null;
    this.b = null;
    this.l = null;
    this.h = null;
    this.bx = null;
    this.ly = null;
    this.ref = null;
    this.mcode1 = null;
    this.mcode2 = null;
    this.mcode3 = null;
  }
  setPid(pid) {
    this.pid = pid;
    return this;
  }
  setBx(bx) {
    this.bx = bx;
    return this;
  }
  setLy(ly) {
    this.ly = ly;
    return this;
  }
  setH(h) {
    this.h = h;
    return this;
  }
  setRef(ref){
    this.ref = ref;
    return this;
  }
  build() {
    return new PointData(this.pid, this.b, this.l, this.h, this.bx, this.ly, this.ref, this.mcode1, this.mcode2, this.mcode3);
  }

}
