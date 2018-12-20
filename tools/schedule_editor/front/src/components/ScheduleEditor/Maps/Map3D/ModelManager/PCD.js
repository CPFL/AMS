import * as THREE from 'three';

/**
 * ADASMapLoader
 */
export default class PCD extends THREE.Group {
  constructor() {
    super();
    this.pcdList = {};
    this.camera = null;
    this.controls = null;
  }

  set3DParameter(camera, controls) {
    this.camera = camera;
    this.controls = controls;
  }

  addPCD(mesh, name) {
    mesh.name = 'PCD/' + name;
    mesh.material.size = 0.1;
    mesh.material.color.setHex(0xffffff);
    this.add(mesh);
    this.pcdList[mesh.name] = mesh;
  }

  deletePCD() {
    for (let i = this.children.length - 1; i >= 0; i--) {
      this.remove(this.children[i]);
    }
    delete this.pcdList;
    this.pcdList = {};
  }

  setPCDMapFromBinary(binaryPCDList) {
    this.deletePCD();
    this.parsePCD(binaryPCDList);
  }

  parsePCD(binaryPCDList) {
    let loader = new THREE.PCDLoader();
    for (let id in binaryPCDList) {
      if (binaryPCDList.hasOwnProperty(id)) {
        let mesh = loader.parse(binaryPCDList[id], 'load_pcd');
        this.addPCD(mesh, id);
      }
    }
  }
}
