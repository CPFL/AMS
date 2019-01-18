import React from 'react';
import PropTypes from 'prop-types';

import connect from 'react-redux/es/connect/connect';
import { bindActionCreators } from 'redux';

import * as THREE from 'three';
import 'three/OrbitControls';
import 'three/PCDLoader';

import Detector from '../../../../../../libs/threejs/Detector';

import PCD from './ModelManager/PCD';
import Waypoint from './ModelManager/Waypoint';

import MapDataUpdater from '../DataUpdater/MapDataUpdater';
import ScheduleListUpdater from '../DataUpdater/ScheduleListUpdater';
import RouteCodeListUpdater from '../DataUpdater/RouteCodeListUpdater';
import RouteCodeUpdater from '../DataUpdater/RouteCodeUpdater';
import RouteCodeAfterChangeRouteUpdater from '../DataUpdater/RouteCodeAfterChangeRouteUpdater';
import ScheduleEditorActiveStepUpdater from '../DataUpdater/ScheduleEditorActiveStepUpdater';
import SelectableDecisionSectionEndPointListUpdater from '../DataUpdater/SelectableDecisionSectionEndPointListUpdater';
import DecisionSectionRouteCodeUpdater from '../DataUpdater/DecisionSectionRouteCodeUpdater';

import * as ScheduleEditorActions from '../../../../../../redux/Actions/ScheduleEditorActions';

import { addResizeListener, removeResizeListener } from 'detect-resize';

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
    this.routeCode = null;
    this.scheduleList = null;
    this.selectableDecisionSectionEndPointList = null;
    this.routeCodeAfterChangeRoute = null;
    this.decisionSectionRouteCode = null;
    this.scheduleEditorActiveStep = null;

    this.initialCameraPosition = { x: 0, y: 0, z: 0 };

    this.PCDManager = new PCD();
    this.waypointsModelManager = new Waypoint();

    this.onClickCanvas = this.onClickCanvas.bind(this);
    this.resize = this.resize.bind(this);
  }

  componentDidMount() {
    this.setMapSize();
    this.prepare();
    this.initMap();

    document
      .getElementById('route_code_map_canvas')
      .addEventListener('click', this.onClickCanvas, false);
    addResizeListener(
      document.getElementById('route_code_map_canvas'),
      this.resize
    );
  }

  componentWillUnmount() {
    this.renderer.forceContextLoss();
    this.renderer.context = this.renderer.domElement = this.renderer = null;

    this.container = this.camera = this.scene = this.controls = this.stats = this.PCDManager = this.waypointsModelManager = this.mapData = this.scheduleList = this.routeCodeAfterChangeRoute = null;
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
    this.waypointsModelManager.setCallback(
      this.props.scheduleEditorActions.setDecisionSectionEndPoint
    );
    this.scene.add(this.waypointsModelManager);

    this.PCDManager.setPCDMapFromBinary(this.mapData.pcd);
    if (
      Object.keys(this.mapData.waypoint).length > 0 &&
      Object.keys(this.mapData.lane).length > 0
    ) {
      this.waypointsModelManager.setWaypoint(
        this.mapData.waypoint,
        this.mapData.lane
      );
    } else {
      this.waypointsModelManager.clear();
    }

    this.waypointsModelManager.init(
      this.scheduleList,
      this.routeCodeList,
      this.routeCode,
      this.routeCodeAfterChangeRoute,
      this.selectableDecisionSectionEndPointList,
      this.decisionSectionRouteCode
    );

    this.waypointsModelManager.setScheduleEditorActiveStep(
      this.scheduleEditorActiveStep
    );
  }

  render() {
    const initMapData = mapData => {
      this.mapData = mapData;
    };

    const initRouteCodeList = routeCodeList => {
      this.routeCodeList = routeCodeList;
    };

    const initRouteCode = routeCode => {
      this.routeCode = routeCode;
    };

    const initScheduleList = scheduleList => {
      this.scheduleList = scheduleList;
    };

    const initSelectableDecisionSectionEndPointList = selectableDecisionSectionEndPointList => {
      this.selectableDecisionSectionEndPointList = selectableDecisionSectionEndPointList;
    };

    const initRouteCodeAfterChangeRoute = routeCodeAfterChangeRoute => {
      this.routeCodeAfterChangeRoute = routeCodeAfterChangeRoute;
    };

    const initDecisionSectionRouteCode = decisionSectionRouteCode => {
      this.decisionSectionRouteCode = decisionSectionRouteCode;
    };

    const initScheduleEditorActiveStep = scheduleEditorActiveStep => {
      this.scheduleEditorActiveStep = scheduleEditorActiveStep;
    };

    const setMapData = mapData => {
      this.PCDManager.setPCDMapFromBinary(mapData.pcd);
      if (
        Object.keys(mapData.waypoint).length > 0 &&
        Object.keys(mapData.lane).length > 0
      ) {
        this.waypointsModelManager.setWaypoint(mapData.waypoint, mapData.lane);
      } else {
        this.waypointsModelManager.clear();
      }
    };

    const updateCurrentRouteCode = routeCode => {
      this.waypointsModelManager.updateCurrentRouteCode(routeCode);
    };

    const updateRouteCodeAfterChangeRoute = routeCodeAfterChangeRoute => {
      this.waypointsModelManager.updateRouteCodeAfterChangeRoute(
        routeCodeAfterChangeRoute
      );
    };

    const setSelectableDecisionSectionEndPointList = selectableDecisionSectionEndPointList => {
      this.waypointsModelManager.setSelectableDecisionSectionEndPointList(
        selectableDecisionSectionEndPointList
      );
    };

    const updateDecisionSectionRouteCode = decisionSectionRouteCode => {
      this.waypointsModelManager.updateDecisionSectionRouteCode(
        decisionSectionRouteCode
      );
    };

    const setScheduleEditorActiveStep = scheduleEditorActiveStep => {
      this.waypointsModelManager.setScheduleEditorActiveStep(
        scheduleEditorActiveStep
      );
    };

    return (
      <div id="route_code_map_canvas" style={{ width: '100%', height: '100%' }}>
        <MapDataUpdater initMapData={initMapData} setMapData={setMapData} />
        <RouteCodeUpdater
          initRouteCode={initRouteCode}
          updateRouteCode={updateCurrentRouteCode}
        />
        <RouteCodeAfterChangeRouteUpdater
          initRouteCodeAfterChangeRoute={initRouteCodeAfterChangeRoute}
          updateRouteCodeAfterChangeRoute={updateRouteCodeAfterChangeRoute}
        />
        <ScheduleListUpdater initScheduleList={initScheduleList} />
        <RouteCodeListUpdater initRouteCodeList={initRouteCodeList} />
        <ScheduleEditorActiveStepUpdater
          initScheduleEditorActiveStep={initScheduleEditorActiveStep}
          setScheduleEditorActiveStep={setScheduleEditorActiveStep}
        />
        <SelectableDecisionSectionEndPointListUpdater
          initSelectableDecisionSectionEndPointList={
            initSelectableDecisionSectionEndPointList
          }
          setSelectableDecisionSectionEndPointList={
            setSelectableDecisionSectionEndPointList
          }
        />
        <DecisionSectionRouteCodeUpdater
          initDecisionSectionRouteCode={initDecisionSectionRouteCode}
          updateDecisionSectionRouteCode={updateDecisionSectionRouteCode}
        />
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
