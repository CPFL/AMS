import React from 'react';

import * as THREE from 'three';
import 'three/OrbitControls';
import 'three/PCDLoader';

import Detector from '../../../../../libs/threejs/Detector';

import PCD from './ModelManager/PCD';
import Waypoint from './ModelManager/Waypoint';

import MapDataUpdater from '../DataUpdater/MapDataUpdater';
import ActiveStepUpdater from '../DataUpdater/ActiveStepUpdater';
import IsBackUpdater from '../DataUpdater/IsBackUpdater';
import RouteCodeUpdater from '../DataUpdater/RouteCodeUpdater';

import connect from 'react-redux/es/connect/connect';
import { bindActionCreators } from 'redux';
import * as ScheduleEditorActions from '../../../../../redux/Actions/ScheduleEditorActions';

import { addResizeListener, removeResizeListener } from 'detect-resize';
import PropTypes from 'prop-types';

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

    this.mapData = null;

    this.initialCameraPosition = { x: 0, y: 0, z: 0 };

    this.PCDManager = new PCD();
    this.waypointsModelManager = new Waypoint();

    this.onClickCanvas = this.onClickCanvas.bind(this);
    this.resize = this.resize.bind(this);
    this.initMapData = this.initMapData.bind(this);
    this.setMapData = this.setMapData.bind(this);
    this.setActiveStep = this.setActiveStep.bind(this);
    this.setIsBack = this.setIsBack.bind(this);
    this.updateRouteCode = this.updateRouteCode.bind(this);
  }

  componentDidMount() {
    console.log('route code map mount');
    this.setMapSize();
    this.prepare();
    this.setMapData(this.mapData);
    document
      .getElementById('route_code_map_canvas')
      .addEventListener('click', this.onClickCanvas, false);
    addResizeListener(
      document.getElementById('route_code_map_canvas'),
      this.resize
    );
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
    delete this.waypointsModelManager;
    delete this.raycaster;
    delete this.mapData;

    this.container = null;
    this.camera = null;
    this.scene = null;
    this.renderer = null;
    this.controls = null;
    this.stats = null;
    this.sceneData = {};
    this.PCDManager = null;
    this.waypointsModelManager = null;
    this.raycaster = null;
    this.mapData = null;

    removeResizeListener(
      document.getElementById('route_code_map_canvas'),
      this.resize
    );
  }

  setMapSize() {
    this.width = document.getElementById('route_code_map_canvas').clientWidth;
    this.height = document.getElementById('route_code_map_canvas').clientHeight;
  }

  onClickCanvas(event) {
    const element = event.currentTarget;
    const elementPosition = element.getBoundingClientRect();

    const x = event.clientX - elementPosition.left;
    const y = event.clientY - elementPosition.top;
    const w = elementPosition.width;
    const h = elementPosition.height;
    this.mouse.x = 2 * (x / w) - 1;
    this.mouse.y = 2 * -(y / h) + 1;

    this.waypointsModelManager.selectObject(this.mouse);
  }

  prepare() {
    this.prepareScene();
    this.initializeCameraPosition();
  }

  resize() {
    this.setMapSize();
    console.log(this.width, this.height);
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
      this.initialCameraPosition.z
    );
  }

  prepareScene() {
    if (this.container === null) {
      this.container = document.getElementById('route_code_map_canvas');
    }

    if (this.camera === null) {
      this.camera = new THREE.PerspectiveCamera(
        27,
        this.width / this.height,
        1,
        10000
      );
      this.camera.up.x = 0;
      this.camera.up.y = 0;
      this.camera.up.z = 1;
      this.scene.add(this.camera);
    }

    if (this.renderer === null) {
      this.renderer = new THREE.WebGLRenderer({ antialias: false });
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
      this.controls = new THREE.OrbitControls(
        this.camera,
        this.renderer.domElement
      );
      this.controls.addEventListener('change', this.renderMap.bind(this));
      this.controls.target.set(0, 0, 0);

      this.animate();
    }

    this.initModels();
  }

  renderMap() {
    this.renderer.render(this.scene, this.camera);
    this.camera.lookAt(this.controls.target);
  }

  initModels() {
    this.PCDManager.set3DParameter(this.camera, this.controls);
    this.scene.add(this.PCDManager);

    this.waypointsModelManager.set3DParameter(this.camera, this.controls);
    this.waypointsModelManager.setCallback(
      this.props.scheduleEditorActions.setStartPointAndLaneList,
      this.props.scheduleEditorActions.setLaneList,
      this.props.scheduleEditorActions.setEndPoint
    );
    this.scene.add(this.waypointsModelManager);
  }

  initMapData(mapData) {
    console.log(mapData);
    this.mapData = mapData;
  }

  setMapData(mapData) {
    console.log(mapData);

    this.PCDManager.setPCDMapFromBinary(mapData.pcd);
    if (
      Object.keys(mapData.waypoint).length > 0 &&
      Object.keys(mapData.lane).length > 0
    ) {
      this.waypointsModelManager.setWaypoint(mapData.waypoint, mapData.lane);
    } else {
      this.waypointsModelManager.clear();
    }
  }

  setActiveStep(activeStep) {
    this.waypointsModelManager.setActiveStep(activeStep);
  }

  setIsBack(isBack) {
    this.waypointsModelManager.setIsBack(isBack);
  }

  updateRouteCode(startPoint, lanes, endPoint) {
    this.waypointsModelManager.updateRouteCode(startPoint, lanes, endPoint);
  }

  render() {
    return (
      <div id="route_code_map_canvas" style={{ width: '100%', height: '100%' }}>
        <MapDataUpdater
          initMapData={this.initMapData}
          setMapData={this.setMapData}
        />
        <ActiveStepUpdater setActiveStep={this.setActiveStep} />
        <IsBackUpdater setIsBack={this.setIsBack} />
        <RouteCodeUpdater updateRouteCode={this.updateRouteCode} />
      </div>
    );
  }
}

Map3DManager.propTypes = {
  scheduleEditorActions: PropTypes.object
};
const mapState = () => ({});

const mapDispatch = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export default connect(
  mapState,
  mapDispatch
)(Map3DManager);
