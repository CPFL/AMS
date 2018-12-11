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
  laneList: null,
  endPoint: "",

  //route code list
  routeCodeList: null,

  //schedule list
  scheduleList: null,

  // 3D View Data
  pcd: {},
  waypoint: {},
  lane: {},

  //Modal
  isImportDataModalOpen: true,
  isAddRouteModalOpen: false

});

export class ScheduleEditor extends ScheduleEditorRecord {

  constructor() {
    super({
      activeStep: "advanceOrBack",
      isBack: false,
      startPoint: "",
      laneList: List(),
      endPoint: "",
      routeCodeList: List(),
      scheduleList: List(),
      pcd: {},
      waypoint: {},
      lane: {},
      isImportDataModalOpen: true,
      isAddRouteModalOpen: false
    });
  }


  //SET

  setActiveStep(activeStep) {
    return this.set('activeStep', activeStep)
  }

  backStep(activeStep) {
    if(activeStep === steps.advanceOrBack.id){
      return this.set('startPoint', "").set('laneList', List())
        .set('endPoint', "").set('isBack', false)
        .set('activeStep', activeStep)
    } else if(activeStep === steps.selectLane.id) {
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

  saveRouteCode(routeCode) {

    const routeCodeList = this.get('routeCodeList').push(routeCode);
    return this.set('startPoint', "").set('laneList', List())
      .set('endPoint', "").set('isBack', false)
      .set('activeStep', steps.advanceOrBack.id).set('routeCodeList', routeCodeList).set('isAddRouteModalOpen', false)
  }

  saveAndAnotherSelectRouteCode(routeCode) {

    const routeCodeList = this.get('routeCodeList').push(routeCode);
    return this.set('startPoint', "").set('laneList', List())
      .set('endPoint', "").set('isBack', false)
      .set('activeStep', steps.advanceOrBack.id).set('routeCodeList', routeCodeList)
  }

  clearRouteCodeData() {
    return this.set('startPoint', "").set('laneList', List()).set('endPoint', "").set('isBack', false)
  }

  setIsImportDataModalOpen(isImportDataModalOpen) {
    return this.set('isImportDataModalOpen', isImportDataModalOpen)
  }

  setIsAddRouteModalOpen(isAddRouteModalOpen) {
    return this.set('isAddRouteModalOpen', isAddRouteModalOpen)
  }


  //GET

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

  getRouteCodeList(){
    return this.get('routeCodeList').toJS();
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

  getIsAddRouteModalOpen() {
    return this.get('isAddRouteModalOpen')
  }

}