import React from 'react';

import * as THREE from 'three';
import 'three/ColladaLoader';
import 'three/OrbitControls';
import 'three/PCDLoader';
import 'three/TDSLoader';

import Detector from "../../../../lib/threejs/Detector";

import ADASMapManager from './ModelManager/ADASMapManager';
import ADASMapLoader from '../../../../io/ADASMap/ADASMapLoader';

import PCD from './ModelManager/PCD';
import Waypoint from './ModelManager/Waypoint';

import MapDataUpdater from '../DataUpdater/MapDataUpdater'
import WidthAndHeightUpdater from "../DataUpdater/WidthAndHeightUpdater";
import ActiveStepUpdater from "../DataUpdater/ActiveStepUpdater";

import {steps} from '../../../../model/Redux/Page/RouteCodeEditor'

import connect from "react-redux/es/connect/connect";
import {bindActionCreators} from "redux";
import * as RouteCodeEditorActions from "../../../../redux/Actions/RouteCodeEditorActions";


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
    this.mouse = new THREE.Vector2();

    this.initialCameraPosition = {x: 3810.814, y: -99443.275, z: 0};

    this.PCDManager = new PCD();
    this.ADASMapManager = new ADASMapManager();
    this.waypointsModelManager = new Waypoint();

  }

  componentDidMount() {
    this.setMapSize(this.props.width, this.props.height);
    this.prepare();
    document.getElementById("map_canvas").addEventListener('click', this.onClickCanvas.bind(this), false);
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
    delete this.PCDManager;
    delete this.ADASMapManager;
    delete this.vehicleModelManager;
    delete this.waypointsModelManager;
    delete this.raycaster;

    this.container = null;
    this.camera = null;
    this.scene = null;
    this.renderer = null;
    this.controls = null;
    this.stats = null;
    this.sceneData = {};
    this.PCDManager = null;
    this.ADASMapManager = null;
    this.vehicleModelManager = null;
    this.waypointsModelManager = null;
    this.raycaster = null;

  }

  setMapSize(width, height) {
    this.width = width ? width : document.getElementById("map_canvas").clientWidth;
    this.height = height ? height : 1000;
    if (this.width > document.getElementById("map_canvas").clientWidth) this.width = document.getElementById("map_canvas").clientWidth;
  }

  onClickCanvas(event) {
    const element = event.currentTarget;
    const elementPosition = element.getBoundingClientRect();

    const x = event.clientX - elementPosition.left;
    const y = event.clientY - elementPosition.top;
    const w = elementPosition.width;
    const h = elementPosition.height;
    this.mouse.x = (x / w) * 2 - 1;
    this.mouse.y = -(y / h) * 2 + 1;

    this.waypointsModelManager.selectObject(this.mouse);

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

    this.PCDManager.set3DParameter(this.camera, this.controls);
    this.scene.add(this.PCDManager);

    this.ADASMapManager = new ADASMapManager();
    this.scene.add(this.ADASMapManager);

    this.waypointsModelManager.set3DParameter(this.camera, this.controls);
    this.waypointsModelManager.setCallback(
      this.props.routeCodeEditorActions.setStartPoint,
      this.props.routeCodeEditorActions.setLaneList,
      this.props.routeCodeEditorActions.setEndPoint,
      this.props.routeCodeEditorActions.clearRouteCodeData
    );
    this.scene.add(this.waypointsModelManager);

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

    /*
    this.adasMapLoader = new ADASMapLoader();
    this.adasMapLoader.fetchADASMap().then(
      adasmap => {
        this.props.SettingsActions.setADASMapFromLocalFile(adasmap);
      });
      */
  }

  setADASMap(adasmap) {
    this.ADASMapManager.loadADASMap(adasmap);
  }

  setMapData(mapData) {

    this.PCDManager.setPCDMapFromBinary(mapData.pcd);

    if (Object.keys(mapData.waypoint).length > 0 && Object.keys(mapData.lane).length > 0) {
      this.waypointsModelManager.setWaypoint(mapData.waypoint, mapData.lane);
    } else {
      this.waypointsModelManager.clear();
    }
  }

  setWidthAndHeight(width, height) {
    this.setMapSize(width, height);
    this.camera.aspect = this.width / this.height;
    this.camera.updateProjectionMatrix();
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(this.width, this.height);
  }

  setActiveStep(activeStep) {
    this.waypointsModelManager.setActiveStep(activeStep);
  }

  render() {
    return (
      <div id="map_canvas">
        <MapDataUpdater
          setMapData={this.setMapData.bind(this)}
        />
        <WidthAndHeightUpdater
          setWidthAndHeight={this.setWidthAndHeight.bind(this)}
        />
        <ActiveStepUpdater
          setActiveStep={this.setActiveStep.bind(this)}
        />
      </div>
    )
  }
}

const mapState = () => ({});


const mapDispatch = (dispatch) => ({
  routeCodeEditorActions: bindActionCreators(RouteCodeEditorActions, dispatch),

});
export default connect(mapState, mapDispatch)(Map3DManager);
