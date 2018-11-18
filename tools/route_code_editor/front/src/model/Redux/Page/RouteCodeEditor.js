import {Record, List} from 'immutable';
import React from "react";


export const steps = {
  advanceOrBack: {
    id: "advanceOrBack",
    name: "Advance or Back",
    previousStep: "",
    nextStep: "selectStartPoint"
  },
  selectStartPoint: {
    id: "selectStartPoint",
    name: "Select Start Point",
    previousStep: "advanceOrBack",
    nextStep: "selectLane"
  },
  selectLane: {
    id: "selectLane",
    name: "Select Lane",
    previousStep: "selectStartPoint",
    nextStep: "selectEndPoint"
  },
  selectEndPoint: {
    id: "selectEndPoint",
    name: "Select End Point",
    previousStep: "selectLane",
    nextStep: "result"
  },
  result: {
    id: "result",
    name: "Result",
    previousStep: "selectEndPoint",
    nextStep: "advanceOrBack"
  }
};

const RouteCodeEditorRecord = new Record({
  mapHeight: 0,
  mapWidth: 0,
  activeStep: "advanceOrBack",
  isBack: false,
  startPoint: "",
  laneList: [],
  endPoint: "",
  pcd: {},
  waypoint: {},
  lane: {}
});

export class RouteCodeEditor extends RouteCodeEditorRecord {

  constructor() {
    super({
      mapHeight: 500,
      mapWidth: 500,
      activeStep: "advanceOrBack",
      isBack: false,
      startPoint: "",
      laneList: [],
      endPoint: "",
      pcd: {},
      waypoint: {},
      lane: {}
    });
  }

  setHeightAndWidth(height, width) {
    return this.set('mapHeight', height).set('mapWidth', width)
  }

  setActiveStep(activeStep) {
    return this.set('activeStep', activeStep)
  }

  setIsBack(isBack) {
    return this.set('isBack', isBack)
  }

  setStartPoint(startPoint){
    return this.set('startPoint', startPoint)
  }

  setLaneList(laneList){
    return this.set('laneList', List(laneList))
  }

  setEndPoint(endPoint){
    return this.set('endPoint', endPoint)
  }

  setMapData(pcd, waypoint, lane){
    return this.set('pcd', pcd).set('waypoint', waypoint).set('lane', lane)
  }

  clearRouteCodeData(){
    return this.set('startPoint', "").set('laneList', List()).set('endPoint', "").set('isBack', false)
  }

  getHeight() {
    return this.get('mapHeight')
  }

  getWidth() {
    return this.get('mapWidth')
  }

  getActiveStep() {
    return this.get('activeStep')
  }

  getIsBack() {
    return this.get('isBack')
  }

  getStartPoint(){
    return this.get('startPoint')
  }

  getLaneList(){
    return this.get('laneList').toJS()
  }

  getEndPoint(){
    return this.get('endPoint')
  }


  getMapData() {
    return {
      pcd: this.get('pcd'),
      waypoint: this.get('waypoint'),
      lane: this.get('lane')
    }
  }

}