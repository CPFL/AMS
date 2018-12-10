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


const ScheduleEditorRecord = new Record({
  // create route code
  activeStep: "advanceOrBack",
  isBack: false,
  startPoint: "",
  laneList: [],
  endPoint: "",

  //route code list
  routeCodeList: [],

  //schedule list
  scheduleList: [],

  // 3D View Data
  pcd: {},
  waypoint: {},
  lane: {},

  //Header
  isImportDataModalOpen: false

});

export class ScheduleEditor extends ScheduleEditorRecord {

  constructor() {
    super({
      activeStep: "advanceOrBack",
      isBack: false,
      startPoint: "",
      laneList: List(),
      endPoint: "",
      pcd: {},
      waypoint: {},
      lane: {},
      isImportDataModalOpen: false
    });
  }

  setActiveStep(activeStep) {
    return this.set('activeStep', activeStep)
  }

  backStep(activeStep) {
    console.log(activeStep);
    if(activeStep === steps.advanceOrBack.id){
      return this.set('startPoint', "").set('laneList', List())
        .set('endPoint', "").set('isBack', false)
        .set('activeStep', activeStep)
    } else if(activeStep === steps.selectLane.id) {
      console.log(activeStep);
      return this.set('activeStep', activeStep).set('endPoint', "")
    }
    return this.set('activeStep', activeStep)
  }

  setIsBack(isBack) {
    return this.set('isBack', isBack)
  }

  setStartPoint(startPoint) {
    return this.set('startPoint', startPoint)
  }

  setLaneList(laneList) {
    return this.set('laneList', List(laneList))
  }

  setEndPoint(endPoint) {
    return this.set('endPoint', endPoint)
  }

  setStartPointAndLaneList(startPoint, laneList) {
    return this.set('startPoint', startPoint).set('laneList', List(laneList))
  }

  setMapData(pcd, waypoint, lane) {
    return this.set('pcd', pcd).set('waypoint', waypoint).set('lane', lane)
  }

  resetRouteCode() {
    return this.set('startPoint', "").set('laneList', List())
      .set('endPoint', "").set('isBack', false)
      .set('activeStep', steps.advanceOrBack.id)
  }

  clearRouteCodeData() {
    return this.set('startPoint', "").set('laneList', List()).set('endPoint', "").set('isBack', false)
  }

  setIsImportDataModalOpen(isImportDataModalOpen) {
    return this.set('isImportDataModalOpen', isImportDataModalOpen)
  }

  getActiveStep() {
    return this.get('activeStep')
  }

  getIsBack() {
    return this.get('isBack')
  }

  getStartPoint() {
    return this.get('startPoint')
  }

  getLaneList() {
    return this.get('laneList').toJS()
  }

  getEndPoint() {
    return this.get('endPoint')
  }

  getMapData() {
    return {
      pcd: this.get('pcd'),
      waypoint: this.get('waypoint'),
      lane: this.get('lane')
    }
  }

  getIsImportDataModalOpen() {
    return this.get('isImportDataModalOpen')
  }


}