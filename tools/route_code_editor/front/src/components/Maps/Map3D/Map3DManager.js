import React from 'react';

import * as THREE from 'three';
import 'three/ColladaLoader';
import 'three/OrbitControls';
import 'three/PCDLoader';
import 'three/TDSLoader';
//import Detector from "threejs/Detector";

import Detector from "../../../lib/threejs/Detector";

import {connect} from "react-redux";
import {bindActionCreators} from "redux";
import * as SettingsActions from "../../../redux/Actions/SettingsActions";

import {OBJECT} from "../../../constants/Constant";

import ADASMapManager from './ModelManager/ADASMapManager';
import ADASMapLoader from '../../../io/ADASMap/ADASMapLoader';

import Object3D from './ModelManager/Object3D';
import PCD from './ModelManager/PCD';
import Vehicle from './ModelManager/Vehicle';
import WaypointsModelManager from './ModelManager/Waypoints';

import VehicleStatusUpdater from "../DataUpdater/VehicleStatusUpdater";
import ShutterStatusUpdater from "../DataUpdater/ShutterStatusUpdater";

import WaypointsUpdater from "../DataUpdater/WaypointsUpdater";
import PCDUpdater from "../DataUpdater/PCDUpdater";
import ADASMapUpdater from "../DataUpdater/ADASMapUpdater";

import CenterVehicleIdUpdater from "../DataUpdater/CenterVehicleIdUpdater";
import MapParameterUpdater from "../DataUpdater/MapParameterUpdater";
import PatoLampStatusUpdater from "../DataUpdater/PatoLampStatusUpdater";


class Map3DManager extends React.Component {


  constructor(props) {
    super(props);
    if (!Detector.webgl) Detector.addGetWebGLMessage();

    this.container = null;
    this.camera = null;
    this.scene = new THREE.Scene();
    this.renderer = null;
    this.controls = null;
    this.stats = null;

    this.sceneData = {
      pointsMap: {},
      vehicleColladaList: {},
      waypointsList: {},

      loadData: {
        loadWaypoints: {}
      }
    };

    this.pcdFileNames = null;
    this.initialCameraPosition = {x: -3215, y: -37394, z: 0};

    this.centerVehicleId = null;
    this.mapParameter = null;

    this.PCDManager = null;
    this.ADASMapManager = null;
    this.vehicleModelManager = null;
    this.waypointsModelManager = null;
    this.object3DManager = null;

    this.patolamps = null;
    this.shutters = null;

  }

  componentDidMount() {
    this.setMapSize(this.props.width, this.props.height);
    this.prepare();
  }

  componentDidUpdate() {
    this.resize()

  }

  componentWillUnmount() {

    delete this.container;
    delete this.stats;
    delete this.camera;
    delete this.renderer;
    delete this.controls;
    delete this.scene;
    delete this.sceneData;
    delete this.pcdFileNames;
    delete this.PCDManager;
    delete this.ADASMapManager;
    delete this.vehicleModelManager;
    delete this.waypointsModelManager;
    delete this.object3DManager;

    this.container = null;
    this.camera = null;
    this.scene = null;
    this.renderer = null;
    this.controls = null;
    this.stats = null;
    this.sceneData = {};
    this.pcdFileNames = null;
    this.PCDManager = null;
    this.ADASMapManager = null;
    this.vehicleModelManager = null;
    this.waypointsModelManager = null;
    this.object3DManager = null;

    window.onresize = null;
  }

  CenterVehicleIdUpdate(vehicleId) {
    this.centerVehicleId = vehicleId;
    if (this.vehicleModelManager !== null) {
      this.vehicleModelManager.updateCenterVehicleId(vehicleId);
    }
  }

  updateMapParameter(param) {
    this.mapParameter = param;
    if (this.vehicleModelManager !== null) {
      this.vehicleModelManager.setIsFollowingCenterVehicle(param.getIsFollowingCenterVehicle());
    }
  }

  setMapSize(width, height) {
    this.width = width ? width : document.getElementById("map_canvas").clientWidth;
    this.height = height ? height : 1000;
    if (this.width > document.getElementById("map_canvas").clientWidth) this.width = document.getElementById("map_canvas").clientWidth;
  }

  reset() {
    delete this.stats;
    delete this.camera;
    delete this.renderer;
    delete this.controls;
    delete this.scene;
    delete this.sceneData;
    delete this.pcdFileNames;
    delete this.PCDManager;
    delete this.ADASMapManager;
    delete this.vehicleModelManager;
    delete this.waypointsModelManager;
    delete this.object3DManager;

    this.container = null;
    this.camera = null;
    this.scene = new THREE.Scene();
    this.renderer = null;
    this.controls = null;
    this.stats = null;
    this.sceneData = {};
    this.pcdFileNames = null;
    this.PCDManager = null;
    this.ADASMapManager = null;
    this.vehicleModelManager = null;
    this.waypointsModelManager = null;
    this.object3DManager = null;

  }

  prepare() {
    this.prepareScene();
    this.initializeCameraPosition();
  }

  resize() {
    this.setMapSize(this.props.width, this.props.height);
    this.camera.aspect = this.width / this.height;
    this.camera.updateProjectionMatrix();
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(this.width, this.height);
  }

  animate() {
    if (this.camera !== null && this.controls !== null) {
      requestAnimationFrame(this.animate.bind(this));
      this.renderMap();
      this.controls.update();
    }
  }

  initializeCameraPosition() {
    this.camera.position.x = this.initialCameraPosition.x;
    this.camera.position.y = this.initialCameraPosition.y;
    this.camera.position.z = this.initialCameraPosition.z + 200;
    this.controls.target.set(
      this.initialCameraPosition.x,
      this.initialCameraPosition.y,
      this.initialCameraPosition.z);
  }


  prepareScene() {
    if (this.container === null) {
      this.container = document.getElementById("map_canvas");

    }

    if (this.camera === null) {
      this.camera = new THREE.PerspectiveCamera(27, this.width / this.height, 1, 10000);
      this.camera.up.x = 0;
      this.camera.up.y = 0;
      this.camera.up.z = 1;
      this.scene.add(this.camera);
    }

    if (this.renderer === null) {
      this.renderer = new THREE.WebGLRenderer({antialias: false});
      this.renderer.setPixelRatio(window.devicePixelRatio);
      this.renderer.setSize(this.width, this.height);
      this.renderer.gammaInput = true;
      this.renderer.gammaOutput = true;
      this.renderer.setClearColor(0x333333, 1.0);

      this.container.appendChild(this.renderer.domElement);
    }

    let light = new THREE.DirectionalLight(0xffffff);
    light.position.set(-1, -1, 1).normalize();
    this.scene.add(light);

    if (this.controls === null) {
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.addEventListener('change', this.renderMap.bind(this));
      this.controls.target.set(0, 0, 0);

      this.animate();
    }

    this.initModels();
    this.initValue();

  }

  renderMap() {
    this.renderer.render(this.scene, this.camera);
    this.camera.lookAt(this.controls.target);

  }

  initModels() {

    this.PCDManager = new PCD(this.camera, this.controls);
    this.scene.add(this.PCDManager);

    this.ADASMapManager = new ADASMapManager();
    this.scene.add(this.ADASMapManager);

    this.vehicleModelManager = new Vehicle(
      this.camera,
      this.controls,
      this.centerVehicleId,
      this.mapParameter.getIsFollowingCenterVehicle());
    this.scene.add(this.vehicleModelManager);

    this.waypointsModelManager = new WaypointsModelManager(this.camera, this.controls);
    this.scene.add(this.waypointsModelManager);

    this.object3DManager = new Object3D();
    this.scene.add(this.object3DManager);
  }

  initValue() {

    /*
    let pcdFileNames = [
        "0.20_Laser-00031_-00375.pcd",
        "0.20_Laser-00032_-00373.pcd",
        "0.20_Laser-00032_-00374.pcd",
        "0.20_Laser-00032_-00375.pcd",
        "0.20_Laser-00032_-00376.pcd",
        "0.20_Laser-00033_-00373.pcd",
        "0.20_Laser-00033_-00374.pcd",
        "0.20_Laser-00033_-00375.pcd",
        "0.20_Laser-00033_-00376.pcd",
        "0.20_Laser-00033_-00377.pcd",
        "0.20_Laser-00034_-00373.pcd",
        "0.20_Laser-00034_-00374.pcd",
        "0.20_Laser-00034_-00375.pcd",
        "0.20_Laser-00034_-00376.pcd",
        "0.20_Laser-00035_-00375.pcd",
        "0.20_Laser-00035_-00376.pcd"
    ];
    this.setPCDFromFile(pcdFileNames);
    */
    this.setObject(OBJECT.OBJECT_LIST);

    this.adasMapLoader = new ADASMapLoader();
    this.adasMapLoader.fetchADASMap().then(
      adasmap => {
        this.props.SettingsActions.setADASMapFromLocalFile(adasmap);
      });
  }

  updateVehicleLocation(vehicleId, vehicleInfo) {
    this.vehicleModelManager.updateVehicleLocation(vehicleId, vehicleInfo);
    //this.waypointsModelManager.emphasisLine(vehicleId, vehicleInfo)
  }

  setWaypoints(waypoints) {
    this.waypointsModelManager.setWaypointsList(waypoints);
  }

  setPCDFromBinary(pcdList) {
    this.PCDManager.setPCDMapFromBinary(pcdList);
  }

  setADASMap(adasmap) {
    this.ADASMapManager.loadADASMap(adasmap);
  }

  setObject(objectData) {
    this.object3DManager.loadObjects(objectData);
    this.object3DManager.loadPatolamps(this.patolamps);
    this.object3DManager.loadObjects(this.shutters);

  }

  initPatoLamps(patolamps) {
    this.patolamps = patolamps;
  }

  updatePatoLampStatus(patolampData) {
    this.object3DManager.updatePatolampStatus(patolampData.id, patolampData.status);
  }

  initShutters(shutters) {
    this.shutters = shutters;
  }

  updateShutterStatus(shutterData) {
    this.object3DManager.updateShutterStatus(shutterData.id, shutterData.status, shutterData.openPercentage);
  }


  render() {
    return (
      <div id="map_canvas">
        <MapParameterUpdater
          updateMapParameter={this.updateMapParameter.bind(this)}
        />
        <VehicleStatusUpdater
          updateVehicleLocation={this.updateVehicleLocation.bind(this)}
        />
        <CenterVehicleIdUpdater
          CenterVehicleIdUpdate={this.CenterVehicleIdUpdate.bind(this)}
        />
        <PatoLampStatusUpdater
          initPatoLamps={this.initPatoLamps.bind(this)}
          updatePatoLampStatus={this.updatePatoLampStatus.bind(this)}
        />
        <ShutterStatusUpdater
          initShutters={this.initShutters.bind(this)}
          updateShutterStatus={this.updateShutterStatus.bind(this)}
        />
        <WaypointsUpdater
          setWaypoints={this.setWaypoints.bind(this)}
        />
        <PCDUpdater
          setPCD={this.setPCDFromBinary.bind(this)}
        />
        <ADASMapUpdater
          setADASMap={this.setADASMap.bind(this)}
        />
      </div>

    )
  }
}


const mapState = (state) => ({
  height: state.routeCodeEditor.getHeight(),
  width: state.routeCodeEditor.getWidth(),
});

const mapDispatch = (dispatch) => ({
  SettingsActions: bindActionCreators(SettingsActions, dispatch)
});

export default connect(mapState, mapDispatch)(Map3DManager);
