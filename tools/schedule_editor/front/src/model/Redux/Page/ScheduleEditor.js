import { Record, List } from 'immutable';

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
  laneList: null,
  endPoint: '',
  isBack: false
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
  currentEditChangeRouoteList: null,

  //create Chage Route
  activeChangeRouteStep: '',
  changeRouteStartPoint: '',
  changeRouteLaneList: null,
  changeRouteEndPoint: '',
  changeRouteIsBack: null,

  //route code list
  routeCodeList: null,

  //schedule list
  scheduleList: null,
  selectableRouteCode: null,

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
      currentEditChangeRouoteList: List(),
      //create Chage Route
      activeChangeRouteStep: '',
      changeRouteStartPoint: '',
      changeRouteLaneList: null,
      changeRouteEndPoint: '',
      changeRouteIsBack: null,
      decisionSectionEndPoint: null,

      //route code list
      routeCodeList: List(),
      //schedule list
      scheduleList: List(),
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

  setActiveStepScheduleEditor(activeStepScheduleEditor) {
    return this.set('activeStepScheduleEditor', activeStepScheduleEditor);
  }

  setStartPointAndLaneList(startPoint, laneList) {
    return this.set('startPoint', startPoint).set('laneList', List(laneList));
  }

  setMapData(pcd, waypoint, lane) {
    return this.set('pcd', pcd)
      .set('waypoint', waypoint)
      .set('lane', lane);
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
      .set('laneList', this.get('laneList'))
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
      .set('laneList', this.get('laneList'))
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

  setIsImportDataModalOpen(isImportDataModalOpen) {
    return this.set('isImportDataModalOpen', isImportDataModalOpen);
  }

  setIsAddRouteModalOpen(isAddRouteModalOpen) {
    return this.set('isAddRouteModalOpen', isAddRouteModalOpen);
  }

  setIsAddScheduleModalOpen(isAddScheduleModalOpen) {
    return this.set('isAddScheduleModalOpen', isAddScheduleModalOpen);
  }

  //GET

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

  getActiveStepScheduleEditor() {
    return this.get('activeStepScheduleEditor');
  }

  getCurrentEditChangeRouoteList() {
    return this.get('currentEditChangeRouoteList');
  }

  getRouteCodeList() {
    return this.get('routeCodeList').toJS();
  }

  getSelectableRouteCode() {
    return this.get('routeCodeList').toJS();
  }

  getMapData() {
    return {
      pcd: this.get('pcd'),
      waypoint: this.get('waypoint'),
      lane: this.get('lane')
    };
  }

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
