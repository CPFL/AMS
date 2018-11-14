import * as THREE from "three";
import 'three/PCDLoader';
import {MAP_3D} from "../../../../constants/Constant";

/**
 * ADASMapLoader
 */
export default class Vehicle extends THREE.Group {

    constructor(camera, controls, centerVehicleId, isFollowingCenterVehicle) {
        super();

        this.manager = new THREE.LoadingManager();
        this.colladaLoader = new THREE.ColladaLoader(this.manager);

        this.camera = camera;
        this.controls = controls;

        this.vehicleList = {};
        this.centerVehicleId = centerVehicleId;
        this.isFollowingCenterVehicle = isFollowingCenterVehicle;

    }

    updateCenterVehicleId(vehicleId) {
        this.centerVehicleId = vehicleId;
    }

    setIsFollowingCenterVehicle(isFollowingCenterVehicle) {
        this.isFollowingCenterVehicle = isFollowingCenterVehicle;
    }

    updateCameraPositionBasedVehicle(newCameraPosition) {
        this.camera.position.x = newCameraPosition.x;
        this.camera.position.y = newCameraPosition.y;
        this.camera.position.z = newCameraPosition.z;

    }

    updateVehicleLocation(vehicleId, vehicleInfo) {
        if (this.controls == null) {
            return
        }

        if (this.vehicleList.hasOwnProperty(vehicleId) && this.vehicleList[vehicleId].isLoaded) {
            if (vehicleId === this.centerVehicleId && this.isFollowingCenterVehicle) {
                this.controls.target.set(vehicleInfo.pose.position.x, vehicleInfo.pose.position.y, vehicleInfo.pose.position.z);

                if (this.cameraFollowVehicleId === this.centerVehicleId) {
                    let x = vehicleInfo.pose.position.x - this.vehicleList[vehicleId].scene.position.x;
                    let y = vehicleInfo.pose.position.y - this.vehicleList[vehicleId].scene.position.y;
                    let z = vehicleInfo.pose.position.z - this.vehicleList[vehicleId].scene.position.z;

                    let newCameraPosition = {
                        x: this.camera.position.x + x,
                        y: this.camera.position.y + y,
                        z: this.camera.position.z + z
                    };
                    this.updateCameraPositionBasedVehicle(newCameraPosition);
                } else {
                    let newCameraPosition = {
                        x: vehicleInfo.pose.position.x,
                        y: vehicleInfo.pose.position.y,
                        z: vehicleInfo.pose.position.z + 200
                    };
                    this.updateCameraPositionBasedVehicle(newCameraPosition);
                    this.cameraFollowVehicleId = this.centerVehicleId;
                }
            }

            this.vehicleList[vehicleId].scene.position.x = vehicleInfo.pose.position.x;
            this.vehicleList[vehicleId].scene.position.y = vehicleInfo.pose.position.y;
            this.vehicleList[vehicleId].scene.position.z = vehicleInfo.pose.position.z;
            this.vehicleList[vehicleId].scene.setRotationFromQuaternion(vehicleInfo.pose.orientation);
        } else {
            this.setVehicleCollada(vehicleId, vehicleInfo);
        }
    }

    setVehicleCollada(vehicleId, vehicleInfo) {

        if(!this.vehicleList.hasOwnProperty(vehicleId)) {
            this.vehicleList[vehicleId] = {
                scene: null,
                isLoaded: false
            };

            this.colladaLoader.options.convertUpAxis = true;

            let path = MAP_3D.MODEL_PATH.VEHICLE.milee_shimz;

            if (MAP_3D.MODEL_PATH.VEHICLE.hasOwnProperty(vehicleInfo.vehicleGroup)) {
                path = MAP_3D.MODEL_PATH.VEHICLE[vehicleInfo.vehicleGroup];
            }

            this.colladaLoader.load(path, (collada) => {

                this.vehicleList[vehicleId].scene = collada.scene;

                this.vehicleList[vehicleId].scene.name = "vehicle/" + vehicleId;
                this.vehicleList[vehicleId].scene.position.x = vehicleInfo.pose.position.x;
                this.vehicleList[vehicleId].scene.position.y = vehicleInfo.pose.position.y;
                this.vehicleList[vehicleId].scene.position.z = vehicleInfo.pose.position.z;
                this.vehicleList[vehicleId].scene.setRotationFromQuaternion(vehicleInfo.pose.orientation);

                this.add(this.vehicleList[vehicleId].scene);
                this.vehicleList[vehicleId].isLoaded = true;

            });
        }
    }

    deleteVehicle() {
        //
    }

}
