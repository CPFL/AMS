import React from 'react';

import * as THREE from 'three';
import 'three/OrbitControls';
import 'three/PCDLoader';

import Detector from '../../../../libs/threejs/Detector';

import PCD from './ModelManager/PCD';
import Waypoint from './ModelManager/Waypoint';

import MapDataUpdater from '../DataUpdater/MapDataUpdater';
import RouteCodeUpdater from '../DataUpdater/RouteCodeUpdater';

import { addResizeListener } from 'detect-resize';

export default class Map3DManager extends React.Component {
  constructor(props) {
    super(props);
    if (!Detector.webgl) Detector.addGetWebGLMessage();

    this.container = this.camera = this.renderer = this.controls = this.stats = this.sceneData = null;

    this.scene = new THREE.Scene();
    this.initialCameraPosition = { x: 0, y: 0, z: 0 };

    this.PCDManager = new PCD();
    this.waypointsModelManager = new Waypoint();

    this.resize = this.resize.bind(this);
    this.setMapData = this.setMapData.bind(this);
    this.updateRouteCode = this.updateRouteCode.bind(this);
  }

  componentDidMount() {
    this.setMapSize();
    this.prepare();
    addResizeListener(document.getElementById('map_canvas'), this.resize);
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
    delete this.initialCameraPosition;

    this.container = this.camera = this.scene = this.renderer = this.controls = this.stats = this.sceneData = this.PCDManager = this.waypointsModelManager = this.initialCameraPosition = null;
  }

  setMapSize() {
    this.width = document.getElementById('map_canvas').clientWidth;
    this.height = document.getElementById('map_canvas').clientHeight;
  }

  prepare() {
    this.prepareScene();
    this.initializeCameraPosition();
  }

  resize() {
    this.setMapSize();
    this.camera.aspect = this.width / this.height;
    this.camera.updateProjectionMatrix();
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(this.width, this.height);
  }

  animate() {
    if (this.camera && this.controls) {
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
      this.container = document.getElementById('map_canvas');
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
    this.scene.add(this.waypointsModelManager);
  }

  setMapData(mapData) {
    this.PCDManager.setPCDMapFromBinary(mapData.pcd);

    if (
      Object.keys(mapData.waypoint).length &&
      Object.keys(mapData.lane).length
    ) {
      this.waypointsModelManager.setWaypoint(mapData.waypoint, mapData.lane);
    } else {
      this.waypointsModelManager.clear();
    }
  }

  updateRouteCode(startPoint, lanes, endPoint) {
    this.waypointsModelManager.updateRouteCode(startPoint, lanes, endPoint);
  }

  render() {
    return (
      <div id="map_canvas" style={{ width: '100%', height: '100%' }}>
        <MapDataUpdater setMapData={this.setMapData} />
        <RouteCodeUpdater updateRouteCode={this.updateRouteCode} />
      </div>
    );
  }
}
