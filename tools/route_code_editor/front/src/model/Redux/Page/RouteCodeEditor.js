import {Record} from 'immutable';
import React from "react";


export const steps = {
  advanceOrBack: {
    id: "advanceOrBack",
    name: "Advance or Back",
    nextTab: "selectStartPoint"
  },
  selectStartPoint: {
    id: "selectStartPoint",
    name: "Select Start Point",
    nextTab: "selectLane"
  },
  selectLane: {
    id: "selectLane",
    name: "Select Lane",
    nextTab: "selectEndPoint"
  },
  selectEndPoint: {
    id: "selectEndPoint",
    name: "Select End Point",
    nextTab: "result"
  },
  result: {
    id: "result",
    name: "Result",
    nextTab: ""
  },
};

const RouteCodeEditorRecord = new Record({
  mapHeight: 0,
  mapWidth: 0,
  activeStep: "advanceOrBack",
  backFlag: false,
  startPointTemp: "",
  laneListTemp: [],
  endPointTemp: "",
  startPoint: "",
  laneList: [],
  endPoint: "",

});

export class RouteCodeEditor extends RouteCodeEditorRecord {

  constructor() {
    super({
      mapHeight: 500,
      mapWidth: 500,
      activeStep: "advanceOrBack",
      isBack: false,
      route_code_temp: {
        startPoint: "",
        laneList: [],
        endPoint: ""
      },
      route_code_confirm: {
        startPoint: "",
        laneList: [],
        endPoint: ""
      }
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


}