export class Vector extends THREE.Mesh{

  constructor(vectorData, point){
    let geometry = new THREE.Geometry();
    geometry.vertices.push(new THREE.Vector3(-0.05, -0.2, 0.05));//0
    geometry.vertices.push(new THREE.Vector3(0.05, -0.2, 0.05));//1
    geometry.vertices.push(new THREE.Vector3(-0.05, 0.1, 0.05));//2
    geometry.vertices.push(new THREE.Vector3(0.05, 0.1, 0.05));//3
    geometry.vertices.push(new THREE.Vector3(-0.05, -0.2, -0.05));//4
    geometry.vertices.push(new THREE.Vector3(0.05, -0.2, -0.05));//5
    geometry.vertices.push(new THREE.Vector3(-0.05, 0.1, -0.05));//6
    geometry.vertices.push(new THREE.Vector3(0.05, 0.1, -0.05));//7
    geometry.vertices.push(new THREE.Vector3(-0.09, 0.1, 0.09));//8
    geometry.vertices.push(new THREE.Vector3(0.09, 0.1, 0.09));//9
    geometry.vertices.push(new THREE.Vector3(0.09, 0.1, -0.09));//10
    geometry.vertices.push(new THREE.Vector3(-0.09, 0.1, -0.09));//11
    geometry.vertices.push(new THREE.Vector3(0, 0.2, 0));
    geometry.faces.push(new THREE.Face3(0, 1, 2));
    geometry.faces.push(new THREE.Face3(1, 3, 2));
    geometry.faces.push(new THREE.Face3(1, 7, 3));
    geometry.faces.push(new THREE.Face3(1, 5, 7));
    geometry.faces.push(new THREE.Face3(0, 4, 1));
    geometry.faces.push(new THREE.Face3(1, 4, 5));
    geometry.faces.push(new THREE.Face3(2, 4, 0));
    geometry.faces.push(new THREE.Face3(2, 6, 4));
    geometry.faces.push(new THREE.Face3(5, 4, 7));
    geometry.faces.push(new THREE.Face3(7, 4, 6));
    geometry.faces.push(new THREE.Face3(8, 11, 9));
    geometry.faces.push(new THREE.Face3(9, 11, 10));
    geometry.faces.push(new THREE.Face3(12, 8, 9));
    geometry.faces.push(new THREE.Face3(12, 11, 8));
    geometry.faces.push(new THREE.Face3(12, 10, 11));
    geometry.faces.push(new THREE.Face3(12, 9, 10));
    geometry.computeFaceNormals();
    super(geometry, new THREE.MeshBasicMaterial({ color: 0xffaa00 }));
    this.defaultColor = 0xffaa00;
    // VectorData.hang, VectorData.vang は時計回り角度、THREE.jsでは回転軸は反時計周り角度なので-1掛ける
    this.rotation.x = -(THREE.Math.degToRad(vectorData.vang - 90));
    this.rotateOnWorldAxis(new THREE.Vector3(0, 0, 1), -(THREE.Math.degToRad(vectorData.hang)));
    this.position.copy(point.position);
    this.data = vectorData;
    this.point = point;
    this.point.setRefferingVector(this);
    this.hilight = false;
    // VectorがPoleから参照されている場合のみセット
    // このときVectorはシーン内では不可視で選択不可能
    // EditPane上での値変更をPoleに移譲する
    this.referringPole = null;
  }
  setReferringPole(pole){
    this.referringPole = pole;
  }
  setHang(hang){
    if(this.referringPole){
      this.referringPole.setHang(hang);
    }else{
      this.rotation.copy(new THREE.Euler(0, 0, 0, 'XYZ'));
      this.rotation.x = -(THREE.Math.degToRad(this.data.vang - 90));
      this.rotateOnWorldAxis(new THREE.Vector3(0, 0, 1), -(THREE.Math.degToRad(hang)));
      this.updateRelatedObjects();
    }
    this.data.hang = hang;
  }
  setVang(vang){
    if(this.referringPole){
      this.referringPole.setVang(vang);
    }else{
      this.rotation.copy(new THREE.Euler(0, 0, 0, 'XYZ'));
      this.rotation.x = -(THREE.Math.degToRad(vang - 90));
      this.rotateOnWorldAxis(new THREE.Vector3(0, 0, 1), -(THREE.Math.degToRad(this.data.hang)));
      this.updateRelatedObjects();
    }
    this.data.vang = vang;
  }

  setBx(bx){
    if(this.referringPole){
      this.referringPole.setBx(bx);
    }else{
      this.position.y = bx;
      this.updateRelatedObjects();
    }
  }
  setLy(ly){
    if(this.referringPole){
      this.referringPole.setLy(ly);
    }else{
      this.position.x = ly;
      this.updateRelatedObjects();
    }
  }
  setH(h){
    if(this.referringPole){
      this.referringPole.setH(h);
    }else{
      this.position.z = h;
      this.updateRelatedObjects();
    }
  }

  /**********************************
   * Object3D model interface methds *
   **********************************/
  updateRelatedObjects() {
    // 関連オブジェクト更新
    this.point.position.copy(this.position);
    this.point.updateRelatedObjects();
  }
  roundDegree(deg){
    return (deg + 360) % 360; 
  }

  enableHighlight(color) {
    let material_color = color || 0xf6ff4c;
    this.material = new THREE.MeshBasicMaterial({ color: material_color });
    this.hilight = true;
  }
  disableHighlight() {
    this.material = new THREE.LineBasicMaterial({ color: this.defaultColor });
    this.hilight = false;
  }
  changeDefaultColor(color) {
    this.defaultColor = color;
    if(this.hilight === false){
      this.material = new THREE.LineBasicMaterial({ color: this.defaultColor });
    }
  }

}

export class VectorData {

  constructor(vid, pid, hang, vang){
    this.vid = parseInt(vid, 10);
    this.pid = parseInt(pid, 10);
    this.hang = parseFloat(hang);
    this.vang = parseFloat(vang);
  }
  toCsv() {
    return this.vid + ',' + this.pid + ',' + this.hang + ',' + this.vang + '\n';
  }
  getCsvHeader(){
    return 'VID,PID,Hang,Vang\n';
  }

}