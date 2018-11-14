/**
 * Basic geometry data
 * Pole - ポール()
 * ref: https://threejs.org/docs/#api/geometries/CylinderGeometry
 */
export class Pole extends THREE.Mesh{

  constructor(pole_data, vector){
    const geometry = new THREE.CylinderGeometry(pole_data.dim / 2, pole_data.dim / 2, pole_data.length);
    geometry.applyMatrix(new THREE.Matrix4().makeTranslation( 0, pole_data.length/2, 0));
    const material = new THREE.MeshBasicMaterial({ color: 0xab4cff });
    material.opacity = 0.8;
    material.transparent = true;
    super(geometry, material);
    this.data = pole_data;
    vector.setReferringPole(this);
    this.vector = vector;
    this.defaultColor = 0xab4cff;
    this.rotation.copy(vector.rotation);
    this.position.copy(vector.position);
    this.hilight = false;
  }

  setLength(length){
    this.data.length = length;
    this.updateGeometry();
  }
  setDim(dim){
    this.data.dim = dim;
    this.updateGeometry();
  }
  updateGeometry(){
    const geometry = new THREE.CylinderGeometry(this.data.dim / 2, this.data.dim / 2, this.data.length);
    geometry.applyMatrix(new THREE.Matrix4().makeTranslation( 0, this.data.length/2, 0));
    this.geometry = geometry;
  }

  setHang(hang){
    this.rotation.copy(new THREE.Euler(0, 0, 0, 'XYZ'));
    this.rotation.x = -(THREE.Math.degToRad(this.vector.data.vang - 90));
    this.rotateOnWorldAxis(new THREE.Vector3(0, 0, 1), -(THREE.Math.degToRad(hang)));
    this.updateRelatedObjects();
  }
  setVang(vang){
    this.rotation.copy(new THREE.Euler(0, 0, 0, 'XYZ'));
    this.rotation.x = -(THREE.Math.degToRad(vang - 90));
    this.rotateOnWorldAxis(new THREE.Vector3(0, 0, 1), -(THREE.Math.degToRad(this.vector.data.hang)));
    this.updateRelatedObjects();
  }

  setBx(bx){
    this.position.y = bx;
    this.updateRelatedObjects();
  }
  setLy(ly){
    this.position.x = ly;
    this.updateRelatedObjects();
  }
  setH(h){
    this.position.z = h;
    this.updateRelatedObjects();
  }

  /**********************************
   * Object3D model interface methds *
   **********************************/
  updateRelatedObjects() {
    // data更新
    this.data.length = this.geometry.parameters.height;
    this.data.dim = this.geometry.parameters.radiusTop * 2;
    // 関連オブジェクト更新
    this.vector.rotation.copy(this.rotation);
    this.vector.position.copy(this.position);
    this.vector.updateRelatedObjects();
  }
  enableHighlight(color) {
    this.material = new THREE.MeshBasicMaterial({ color: color || 0xf6ff4c });
    this.hilight = true;
  }
  disableHighlight() {
    this.material = new THREE.MeshBasicMaterial({ color: this.defaultColor });
    this.hilight = false;
  }
  changeDefaultColor(color){
    this.defaultColor = color;
    if(this.hilight === false){
      this.material = new THREE.MeshBasicMaterial({ color: this.defaultColor });
    }
  }

}

export class PoleData {

  constructor(plid, vid, length, dim){
    this.plid = parseInt(plid, 10);
    this.vid = parseInt(vid, 10);
    this.length = parseFloat(length);
    this.dim = parseFloat(dim);// diameter 直径
  }
  toCsv() {
    return this.plid + ',' + this.vid + ',' + this.length + ',' + this.dim + '\n';
  }
  getCsvHeader(){
    return 'PLID,VID,Length,Dim\n';
  }

}