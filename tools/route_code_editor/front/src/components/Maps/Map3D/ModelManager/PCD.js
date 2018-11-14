import * as THREE from "three";

/**
 * ADASMapLoader
 */
export default class PCD extends THREE.Group {

    constructor(camera, controls) {
        super();
        this.pcdList ={};
        this.camera = camera;
        this.controls = controls;

        this.initialCameraPosition = {x: -3215, y: -37394, z: 0};

    }

    updateCameraPositionBasedMap(pointsMapViewPosition) {
        this.camera.position.x = pointsMapViewPosition.x;
        this.camera.position.y = pointsMapViewPosition.y;
        this.camera.position.z = pointsMapViewPosition.z + 200;
        this.controls.target.set(
            pointsMapViewPosition.x,
            pointsMapViewPosition.y,
            pointsMapViewPosition.z);
    }

    addPCD(mesh, name){
        mesh.name = "PCD/" + name;
        mesh.material.size = 0.1;
        mesh.material.color.setHex(0xffffff);
        this.add(mesh);
        this.pcdList[mesh.name] = mesh;
    }

    deletePCD(){
        for (let i = this.children.length - 1; i >= 0; i--) {
            this.remove(this.children[i]);
        }
        delete this.pcdList;
        this.pcdList = {};
    }

    setPCDFromFile(FileList) {
        this.deletePCD();
        this.loadPCDFromFile(FileList);
    }

    addPCDFromFile(FileList) {
        this.loadPCDFromFile(FileList);
    }

    loadPCDFromFile(FileList) {
        let updateCameraPositionFlag = true;
        const setCameraPosition = (args) => {
            if (updateCameraPositionFlag) {
                let pointsMapViewPosition = {
                    x: args.geometry.attributes.position.array[0],
                    y: args.geometry.attributes.position.array[1],
                    z: args.geometry.attributes.position.array[2]
                };
                this.updateCameraPositionBasedMap(pointsMapViewPosition);

                updateCameraPositionFlag = false;
            }
        };

        let loader = new THREE.PCDLoader();
        for (let pcdFileName of FileList) {
            loader.load("/static/data/maps/shimz/points/" + pcdFileName,
                (mesh) => {
                    this.addPCD(mesh, pcdFileName);
                    setCameraPosition(mesh);
                }
            );
        }
    }

    setPCDMapFromBinary(binaryPCDList){
        this.deletePCD();
        this.parsePCD(binaryPCDList);
    }

    addPCDMapFromBinary(binaryPCDList){
        this.parsePCD(binaryPCDList);
    }

    parsePCD(binaryPCDList) {

        let updateCameraPositionFlag = true;
        const setCameraPosition = (args) => {
            if (updateCameraPositionFlag) {
                let pointsMapViewPosition = {
                    x: args.geometry.attributes.position.array[0],
                    y: args.geometry.attributes.position.array[1],
                    z: args.geometry.attributes.position.array[2]
                };
                this.updateCameraPositionBasedMap(pointsMapViewPosition);

                updateCameraPositionFlag = false;
            }
        };

        let loader = new THREE.PCDLoader();
        for (let id in binaryPCDList) {
            if (binaryPCDList.hasOwnProperty(id)) {
                let mesh = loader.parse(binaryPCDList[id], "load_pcd");
                this.addPCD(mesh, id);
                setCameraPosition(mesh);
            }
        }

    }

}
