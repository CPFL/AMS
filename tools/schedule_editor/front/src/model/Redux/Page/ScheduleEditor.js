import { Record, List, Map } from 'immutable';

export const steps = {
  advanceOrBack: {
    id: 'advanceOrBack',
    name: 'Advance or Back',
    previousStep: '',
    nextStep: 'selectStartPoint'
  },
  selectStartPoint: {
    id: 'selectStartPoint',
    name: 'Select Start Point',
    previousStep: 'advanceOrBack',
    nextStep: 'selectLane'
  },
  selectLane: {
    id: 'selectLane',
    name: 'Select Lane',
    previousStep: 'selectStartPoint',
    nextStep: 'selectEndPoint'
  },
  selectEndPoint: {
    id: 'selectEndPoint',
    name: 'Select End Point',
    previousStep: 'selectLane',
    nextStep: 'result'
  },
  result: {
    id: 'result',
    name: 'Result',
    previousStep: 'selectEndPoint',
    nextStep: 'advanceOrBack'
  }
};

export const scheduleEditSteps = {
  selectRouteCode: {
    id: 'selectRouteCode',
    name: 'Select Route Code',
    nextStep: ['selectChangeRoute', 'result']
  },
  changeRouteEditor: {
    id: 'changeRouteEditor',
    name: 'Change Route Editor',
    nextStep: ['selectRouteCode']
  },
  selectChangeRoute: {
    id: 'selectChangeRoute',
    name: 'Select Change Route',
    nextStep: ['result']
  },
  result: {
    id: 'result',
    name: 'Result',
    nextStep: []
  }
};

const RouteCodeRecord = new Record({
  routeCode: '',
  startPoint: '',
  laneList: [],
  endPoint: '',
  isBack: false
});

const ScheduleRecord = new Record({
  routeCode: '',
  startPoint: '',
  laneList: [],
  endPoint: '',
  isBack: false,
  sendEngage: false,
  waitTime: 0,
  changeRouteList: null,
  lastRouteCode: null
});

const ScheduleEditorRecord = new Record({
  // create route code
  activeStep: '',
  startPoint: '',
  laneList: null,
  endPoint: '',
  isBack: null,

  // create schedule
  activeStepScheduleEditor: '',
  selectableRouteCodeList: null,
  currentRouteCodeSchedule: '',
  checkedSendEngage: false,
  waitTime: 0,
  currentEditChangeRouteList: null,

  //create Change Route
  changeRouteActiveStep: 0,
  changeRouteStartPoint: '',
  changeRouteLaneList: null,
  changeRouteEndPoint: '',
  changeRouteIsBack: null,

  //route code list
  routeCodeList: null,

  //schedule list
  scheduleList: null,
  lastRoute: null,

  // 3D View Data
  pcd: {},
  waypoint: {},
  lane: {},

  //Modal
  isImportDataModalOpen: true,
  isAddRouteModalOpen: false,
  isAddScheduleModalOpen: false
});

export class ScheduleEditor extends ScheduleEditorRecord {
  constructor() {
    super({
      // create route code
      activeStep: 'advanceOrBack',
      isBack: false,
      startPoint: '',
      laneList: List(),
      endPoint: '',
      // create schedule
      activeStepScheduleEditor: 'selectRouteCode',
      selectableRouteCodeList: List(),
      currentRouteCodeSchedule: new RouteCodeRecord(),
      checkedSendEngage: false,
      waitTime: 0,
      currentEditChangeRouteList: List(),
      currentLastRoute: new RouteCodeRecord(),

      //create Change Route
      changeRouteActiveStep: 0,
      changeRouteStartPoint: '',
      changeRouteLaneList: null,
      changeRouteEndPoint: '',
      changeRouteIsBack: null,
      decisionSectionEndPoint: null,

      //route code list
      routeCodeList: List(),
      //schedule list
      scheduleList: List(),
      lastRoute: new RouteCodeRecord(),

      // 3D View Data
      pcd: {},
      waypoint: {},
      lane: {},
      //Modal
      isImportDataModalOpen: true,
      isAddRouteModalOpen: false,
      isAddScheduleModalOpen: false
    });
  }

  //SET

  //Route Code Editor
  setActiveStep(activeStep) {
    return this.set('activeStep', activeStep);
  }

  backStep(activeStep) {
    if (activeStep === steps.advanceOrBack.id) {
      return this.set('startPoint', '')
        .set('laneList', List())
        .set('endPoint', '')
        .set('isBack', false)
        .set('activeStep', activeStep);
    } else if (activeStep === steps.selectLane.id) {
      return this.set('activeStep', activeStep).set('endPoint', '');
    }
    return this.set('activeStep', activeStep);
  }

  setIsBack(isBack) {
    return this.set('isBack', isBack);
  }

  setStartPoint(startPoint) {
    return this.set('startPoint', startPoint);
  }

  setLaneList(laneList) {
    return this.set('laneList', List(laneList));
  }

  setEndPoint(endPoint) {
    return this.set('endPoint', endPoint);
  }

  setStartPointAndLaneList(startPoint, laneList) {
    return this.set('startPoint', startPoint).set('laneList', List(laneList));
  }

  resetRouteCode() {
    return this.set('startPoint', '')
      .set('laneList', List())
      .set('endPoint', '')
      .set('isBack', false)
      .set('activeStep', steps.advanceOrBack.id);
  }

  saveRouteCode(routeCode) {
    const routeCodeRecord = new RouteCodeRecord()
      .set('startPoint', this.get('startPoint'))
      .set('laneList', this.get('laneList').toJS())
      .set('endPoint', this.get('endPoint'))
      .set('isBack', this.get('isBack'))
      .set('routeCode', routeCode);
    const routeCodeList = this.get('routeCodeList').push(routeCodeRecord);
    return this.set('startPoint', '')
      .set('laneList', List())
      .set('endPoint', '')
      .set('isBack', false)
      .set('activeStep', steps.advanceOrBack.id)
      .set('routeCodeList', routeCodeList)
      .set('isAddRouteModalOpen', false);
  }

  saveAndAnotherSelectRouteCode(routeCode) {
    const routeCodeRecord = new RouteCodeRecord()
      .set('startPoint', this.get('startPoint'))
      .set('laneList', this.get('laneList').toJS())
      .set('endPoint', this.get('endPoint'))
      .set('isBack', this.get('isBack'))
      .set('routeCode', routeCode);

    const routeCodeList = this.get('routeCodeList').push(routeCodeRecord);
    return this.set('startPoint', '')
      .set('laneList', List())
      .set('endPoint', '')
      .set('isBack', false)
      .set('activeStep', steps.advanceOrBack.id)
      .set('routeCodeList', routeCodeList);
  }

  clearRouteCodeData() {
    return this.set('startPoint', '')
      .set('laneList', List())
      .set('endPoint', '')
      .set('isBack', false);
  }

  //Change Route
  setChangeRouteActiveStepNext(changeRouteActiveStep) {
    console.log(changeRouteActiveStep);
    changeRouteActiveStep = changeRouteActiveStep + 1;

    return this.set('changeRouteActiveStep', changeRouteActiveStep);
  }

  setChangeRouteActiveStepPrevious(changeRouteActiveStep) {
    changeRouteActiveStep = changeRouteActiveStep - 1;
    if (changeRouteActiveStep < 0) {
      changeRouteActiveStep = 0;
    }
    return this.set('changeRouteActiveStep', changeRouteActiveStep);
  }

  setChangeRouteActiveStepReset() {
    return this.set('changeRouteActiveStep', 0);
  }

  setChangeRouteIsBack(changeRouteIsBack) {
    return this.set('changeRouteIsBack', changeRouteIsBack);
  }

  setChangeRouteStartPoint(changeRouteStartPoint) {
    return this.set('changeRouteStartPoint', changeRouteStartPoint);
  }

  setChangeRouteLaneList(changeRouteLaneList) {
    return this.set('changeRouteLaneList', List(changeRouteLaneList));
  }

  setChangeRouteEndPoint(changeRouteEndPoint) {
    return this.set('changeRouteEndPoint', changeRouteEndPoint);
  }

  setDecisionSectionEndPoint(decisionSectionEndPoint) {
    return this.set('decisionSectionEndPoint', decisionSectionEndPoint);
  }

  //Schedule Editor
  setActiveStepScheduleEditor(activeStepScheduleEditor) {
    return this.set('activeStepScheduleEditor', activeStepScheduleEditor);
  }

  setCurrentRouteCodeSchedule(currentRouteCodeSchedule) {
    let routeCode = null;
    if (currentRouteCodeSchedule !== undefined) {
      routeCode = new RouteCodeRecord(currentRouteCodeSchedule);
    } else {
      routeCode = new RouteCodeRecord();
    }
    console.log(routeCode);
    return this.set('currentRouteCodeSchedule', routeCode).set(
      'currentLastRoute',
      routeCode
    );
  }

  setCheckedSendEngage(checkedSendEngage) {
    return this.set('checkedSendEngage', checkedSendEngage);
  }

  setWaitTime(waitTime) {
    return this.set('waitTime', waitTime);
  }

  setCurrentEditChangeRouteList(currentEditChangeRouteList) {
    return this.set('currentEditChangeRouteList', currentEditChangeRouteList);
  }

  //3D Map
  setMapData(pcd, waypoint, lane) {
    return this.set('pcd', pcd)
      .set('waypoint', waypoint)
      .set('lane', lane);
  }

  //Modal
  setIsImportDataModalOpen(isImportDataModalOpen) {
    return this.set('isImportDataModalOpen', isImportDataModalOpen);
  }

  setIsAddRouteModalOpen(isAddRouteModalOpen) {
    return this.set('isAddRouteModalOpen', isAddRouteModalOpen);
  }

  setIsAddScheduleModalOpen(isAddScheduleModalOpen) {
    const lastRoute = this.get('lastRoute');
    if (lastRoute.routeCode === '') {
      return this.set('isAddScheduleModalOpen', isAddScheduleModalOpen).set(
        'selectableRouteCodeList',
        this.get('routeCodeList')
      );
    } else {
      const RouteCodeList = this.get('routeCodeList').toJS();
      const selectableRouteCodeList = List();
      for (const routeCode of RouteCodeList) {
        const endPoint = lastRoute.endPoint;
        const routeStartPoint = routeCode.startPoint;
        if (endPoint === routeStartPoint) {
          selectableRouteCodeList.push(routeCode);
        }
      }
      selectableRouteCodeList.push(this.get('currentRouteCodeSchedule'));
      return this.set('isAddScheduleModalOpen', isAddScheduleModalOpen).set(
        'selectableRouteCodeList',
        selectableRouteCodeList
      );
    }
  }

  //GET

  //Route Code Editor
  getActiveStep() {
    return this.get('activeStep');
  }

  getIsBack() {
    return this.get('isBack');
  }

  getStartPoint() {
    return this.get('startPoint');
  }

  getLaneList() {
    return this.get('laneList').toJS();
  }

  getEndPoint() {
    return this.get('endPoint');
  }

  //Change Route Code
  getChangeRouteActiveStep() {
    return this.get('changeRouteActiveStep');
  }

  getChangeRouteIsBack() {
    return this.get('changeRouteIsBack');
  }

  getChangeRouteStartPoint() {
    return this.get('changeRouteStartPoint');
  }

  getChangeRouteLaneList() {
    return this.get('changeRouteLaneList').toJS();
  }

  getChangeRouteEndPoint() {
    return this.get('changeRouteEndPoint');
  }

  //Schedule Editor
  getActiveStepScheduleEditor() {
    return this.get('activeStepScheduleEditor');
  }

  getSelectableRouteCode() {
    return this.get('selectableRouteCodeList').toJS();
  }

  getCurrentRouteCodeSchedule() {
    console.log(this.get('currentRouteCodeSchedule').toJS());
    return this.get('currentRouteCodeSchedule').toJS();
  }

  getCheckedSendEngage() {
    return this.get('checkedSendEngage');
  }

  getWaitTime() {
    return this.get('waitTime');
  }

  getCurrentEditChangeRouteList() {
    return this.get('currentEditChangeRouteList');
  }

  //Route List
  getRouteCodeList() {
    return this.get('routeCodeList').toJS();
  }

  getMapData() {
    return {
      pcd: this.get('pcd'),
      waypoint: this.get('waypoint'),
      lane: this.get('lane')
    };
  }

  //Modal
  getIsImportDataModalOpen() {
    return this.get('isImportDataModalOpen');
  }

  getIsAddRouteModalOpen() {
    return this.get('isAddRouteModalOpen');
  }

  getIsAddScheduleModalOpen() {
    return this.get('isAddScheduleModalOpen');
  }
}
