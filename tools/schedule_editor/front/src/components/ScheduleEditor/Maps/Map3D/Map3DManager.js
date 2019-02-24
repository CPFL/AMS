import React from 'react';
import { addResizeListener } from 'detect-resize';

import * as THREE from 'three';
import 'three/OrbitControls';
import 'three/PCDLoader';

import { Detector } from '../../../../libs/threejs/Detector';

import PCD from './ModelManager/PCD';
import Waypoint from './ModelManager/Waypoint';

import MapDataUpdater from '../DataUpdater/MapDataUpdater';
import SelectedDisplayRouteMainViewerUpdater from '../DataUpdater/SelectedDisplayRouteMainViewerUpdater';

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
  }

  componentDidMount() {
    this.setMapSize();
    this.prepare();
    this.initMap();
    addResizeListener(document.getElementById('map_canvas'), this.resize);
  }

  componentWillUnmount() {
    this.renderer.forceContextLoss();
    this.renderer.context = null;
    this.renderer.domElement = null;
    this.renderer = null;

    this.container = this.camera = this.scene = this.controls = this.stats = this.sceneData = this.PCDManager = this.waypointsModelManager = this.initialCameraPosition = null;
  }

  setMapSize() {
    this.width = document.getElementById('map_canvas').clientWidth;
    this.height = document.getElementById('map_canvas').clientHeight;
  }

  resize() {
    this.setMapSize();
    this.camera.aspect = this.width / this.height;
    this.camera.updateProjectionMatrix();
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(this.width, this.height);
  }

  prepare() {
    this.prepareScene();
    this.initializeCameraPosition();
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

    if (this.controls === null) {
      this.controls = new THREE.OrbitControls(
        this.camera,
        this.renderer.domElement
      );
      this.controls.addEventListener('change', this.renderMap.bind(this));
      this.controls.target.set(0, 0, 0);

      this.animate();
    }
  }

  renderMap() {
    this.renderer.render(this.scene, this.camera);
    this.camera.lookAt(this.controls.target);
  }

  initMap() {
    this.PCDManager.set3DParameter(this.camera, this.controls);
    this.scene.add(this.PCDManager);
    this.waypointsModelManager.set3DParameter(this.camera, this.controls);
    this.scene.add(this.waypointsModelManager);
  }

  render() {
    const setMapData = mapData => {
      this.PCDManager.setPCDMapFromBinary(mapData.pcd);
      if (
        Object.keys(mapData.waypoint).length &&
        Object.keys(mapData.lane).length
      ) {
        this.waypointsModelManager.setWaypoint(mapData.waypoint, mapData.lane);
      } else {
        this.waypointsModelManager.clear();
      }
    };
    const updateSelectedDisplayRouteMainViewer = selectedDisplayRouteMainViewer => {
      this.waypointsModelManager.updateSelectedDisplayRouteMainViewer(
        selectedDisplayRouteMainViewer
      );
    };

    return (
      <div id="map_canvas" style={{ width: '100%', height: '100%' }}>
        <MapDataUpdater setMapData={setMapData} />
        <SelectedDisplayRouteMainViewerUpdater
          updateSelectedDisplayRouteMainViewer={
            updateSelectedDisplayRouteMainViewer
          }
        />
      </div>
    );
  }
}
