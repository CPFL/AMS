import * as THREE from "three";
import 'three/TDSLoader';

/**
 * ADASMapLoader
 */
export default class Object3D extends THREE.Group {

    constructor() {
        super();
        this.objectList = {};

        this.individualFunctionList = {
            shutterOpenPercentage: this.updateShutterOpenPercentage
        }

    }

    loadObjects(dataList) {

        let loader = new THREE.TDSLoader();
        for (let id in dataList) {
            if (dataList.hasOwnProperty(id)) {
                const objectData = dataList[id];
                loader.load(objectData.modelPath, (object) => {
                    this.addObject(object, objectData);
                });
            }
        }
    }

    loadPatolamps(dataList) {
        let loader = new THREE.TDSLoader();
        for (let id in dataList) {
            if (dataList.hasOwnProperty(id)) {
                const objectData = dataList[id];
                loader.load(objectData.modelPath, (object) => {
                    this.addLight(object, objectData);
                });
            }
        }
    }

    addObject(object, objectData) {
        object.scale.set(objectData.scale.x, objectData.scale.y, objectData.scale.z);
        object.setRotationFromAxisAngle(new THREE.Vector3(0, 0, 1), objectData.rotationZ);
        object.position.set(objectData.position.x, objectData.position.y, objectData.position.z);
        object.name = "object/" + objectData.id;
        this.initIndividualObjectState(object, objectData);

        console.log(object);

        this.add(object);
        this.objectList[objectData.id] = object;
    }

    addLight(object, objectData) {

        let name = objectData.id;
        let position = objectData.position;
        let scale = objectData.scale;

        const spotLight1 = new THREE.SpotLight(new THREE.Color(0xFF9999), 0.1, 1.0, Math.PI * 2 / 3, 0, 3);
        spotLight1.position.set(position.x, position.y, position.z + 0.5);
        spotLight1.name = "object/" + name + "/spotLight1";
        const spotLight2 = new THREE.SpotLight(new THREE.Color(0xFF9999), 0.1, 1.0, Math.PI * 2 / 3, 0, 3);
        spotLight2.position.set(position.x, position.y, position.z + 0.5);
        spotLight2.name = "object/" + name + "/spotLight2";

        const backMaterial = new THREE.MeshPhysicalMaterial({
            map: null,
            color: new THREE.Color(0xff0000),
            metalness: 1.0,
            roughness: 0.5,
            opacity: 0.75,
            side: THREE.BackSide,
            transparent: true,
            premultipliedAlpha: true
        });
        const frontMaterial = new THREE.MeshPhysicalMaterial({
            map: null,
            color: new THREE.Color(0xff0000),
            metalness: 0.0,
            roughness: 0,
            opacity: 0.25,
            side: THREE.FrontSide,
            transparent: true,
            premultipliedAlpha: true
        });

        object.add(object.children[0].clone());
        object.children[0].material = backMaterial;
        object.children[1].material.side = THREE.DoubleSide;
        object.children[1].material.color = new THREE.Color(0x555555);
        object.children[2].material = frontMaterial;
        object.scale.set(scale.x, scale.y, scale.z);
        object.position.set(position.x, position.y, position.z);
        object.name = "object/" + name;

        this.initIndividualObjectState(object, objectData);

        this.objectList[name] = {};
        this.objectList[name].scene = object;
        this.objectList[name].spotLights = [spotLight1, spotLight2];
        this.objectList[name].state = "off";

        this.add(this.objectList[name].scene);
        this.add(this.objectList[name].spotLights[0]);
        this.add(this.objectList[name].spotLights[0].target);
        this.add(this.objectList[name].spotLights[1]);
        this.add(this.objectList[name].spotLights[1].target);
    }

    initIndividualObjectState(object, objectData) {
        if (objectData.hasOwnProperty("initState")) {
            for (let state of objectData.initState) {
                if (this.individualFunctionList.hasOwnProperty(state.type)) {
                    this.individualFunctionList[state.type](state, object);
                }
            }
        }
    }

    updateShutterStatus(shutterId, status, percentage) {

        const object = this.getObjectByName("object/" + shutterId);
        const state = {
            percentage: percentage
        };

        if(object !== undefined) {
            this.updateShutterOpenPercentage(state, object)
        }
    }


    updateShutterOpenPercentage(state, object) {
        let openPercentage = state.percentage;
        // open_percentage: 0.0~1.0

        if (openPercentage > 1.0) {
            openPercentage = 1.0;
        } else if (openPercentage < 0) {
            openPercentage = 0;
        }

        object.children[2].position.z = object.position.z + 2400000 * 0.001 * openPercentage;  // 1.0: 0, 0.75:600, 0.5:1200, 0.25:1800, 0.0:2400
        object.children[2].scale.z = Math.max(0.00001, 1.0 - openPercentage);
    }

    updatePatolampStatus(patolampId, patolampStatus) {

        if (patolampStatus === "on" && this.objectList[patolampId].state === "off") {
            this.objectList[patolampId].spotLights[0].intensity = 300.0;
            this.objectList[patolampId].spotLights[1].intensity = 300.0;
            this.objectList[patolampId].state = "on";
            this.rotatePatolamp(patolampId);
        }
        else if (patolampStatus === "off") {
            this.objectList[patolampId].spotLights[0].intensity = 0.01;
            this.objectList[patolampId].spotLights[1].intensity = 0.01;
            this.objectList[patolampId].state = "off";
        }
    }

    rotatePatolamp(patolampId) {

        let waitFunc = () => {
            if (this.objectList[patolampId].state === "on") {
                const spotLights = this.objectList[patolampId].spotLights;
                const date = new Date();
                const ms = date.getTime() / 100;
                spotLights[0].target.position.set(
                    spotLights[0].position.x + Math.cos(ms),
                    spotLights[0].position.y + Math.sin(ms),
                    spotLights[0].position.z
                );
                spotLights[1].target.position.set(
                    spotLights[1].position.x + Math.cos(ms + Math.PI),
                    spotLights[1].position.y + Math.sin(ms + Math.PI),
                    spotLights[1].position.z
                );
                clearTimeout(id);
                id = setTimeout(waitFunc, 100);
            }
        };

        let id = setTimeout(waitFunc, 100);
    }


}
