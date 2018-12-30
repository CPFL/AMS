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
  checkedSendEngage: false,
  waitTime: 0,
  changeRouteList: null,
  lastRoute: null
});

const selectedDisplayRouteMainViewerRecord = new Record({
  type: '',
  selectRoute: new RouteCodeRecord(),
  routeList: List()
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
  nextSelectableRouteCodeList: null,
  currentRouteCodeSchedule: null,
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
  selectedDisplayRouteMainViewer: null,
  selectRouteCodeDisplayMainViewer: null,
  selectScheduleDisplayMainViewer: null,

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
  isAddRouteByTextModalOpen: false,
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
      nextSelectableRouteCodeList: List(),
      currentRouteCodeSchedule: new RouteCodeRecord(),
      checkedSendEngage: false,
      waitTime: 0,
      currentEditChangeRouteList: List(),

      //create Change Route
      changeRouteActiveStep: 0,
      changeRouteStartPoint: '',
      changeRouteLaneList: null,
      changeRouteEndPoint: '',
      changeRouteIsBack: null,
      decisionSectionEndPoint: null,

      //route code list
      routeCodeList: List(),
      selectedDisplayRouteMainViewer: new selectedDisplayRouteMainViewerRecord(),
      selectRouteCodeDisplayMainViewer: new RouteCodeRecord(),
      selectScheduleDisplayMainViewer: new RouteCodeRecord(),

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
      isAddRouteByTextModalOpen: false,
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

  //Schedule Editor
  setActiveStepScheduleEditor(activeStepScheduleEditor) {
    return this.set('activeStepScheduleEditor', activeStepScheduleEditor);
  }

  setCurrentRouteCodeSchedule(currentRouteCodeSchedule) {
    let currentRouteCode = null;
    if (currentRouteCodeSchedule) {
      currentRouteCode = new RouteCodeRecord(currentRouteCodeSchedule);
    } else {
      currentRouteCode = new RouteCodeRecord();
    }
    return this.set('currentRouteCodeSchedule', currentRouteCode);
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

  saveSchedule() {
    const selectRouteCode = this.get('currentRouteCodeSchedule');
    if (selectRouteCode.routeCode !== '') {
      const lastRoute = selectRouteCode;
      const newSchedule = new ScheduleRecord()
        .set('routeCode', selectRouteCode.routeCode)
        .set('startPoint', selectRouteCode.startPoint)
        .set('laneList', selectRouteCode.laneList)
        .set('endPoint', selectRouteCode.endPoint)
        .set('isBack', selectRouteCode.isBack)
        .set('checkedSendEngage', this.get('checkedSendEngage'))
        .set('waitTime', this.get('waitTime'))
        .set('lastRoute', lastRoute);
      const newScheduleList = this.get('scheduleList').push(newSchedule);
      return this.set('scheduleList', newScheduleList)
        .set('lastRoute', lastRoute)
        .set('activeStepScheduleEditor', 'selectRouteCode')
        .set('currentRouteCodeSchedule', new RouteCodeRecord())
        .set('checkedSendEngage', false)
        .set('waitTime', 0)
        .set('currentEditChangeRouteList', List())
        .set('isAddScheduleModalOpen', false);
    } else {
      return this.set('isAddScheduleModalOpen', false)
        .set('activeStepScheduleEditor', 'selectRouteCode')
        .set('currentRouteCodeSchedule', new RouteCodeRecord())
        .set('checkedSendEngage', false)
        .set('waitTime', 0)
        .set('currentEditChangeRouteList', List())
        .set('scheduleList', List())
        .set('lastRoute', new RouteCodeRecord());
    }
  }

  //Change Route
  setChangeRouteActiveStepNext(changeRouteActiveStep) {
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

  //Route Code List
  deleteRouteCodeFromRouteCodeListByIndex(index) {
    const deleteRecord = this.get('routeCodeList')
      .get(index)
      .toJS();
    const selectedDisplayRouteMainViewer = this.get(
      'selectedDisplayRouteMainViewer'
    ).toJS().selectRoute;

    console.log(deleteRecord, selectedDisplayRouteMainViewer);
    if (deleteRecord.routeCode === selectedDisplayRouteMainViewer.routeCode) {
      return this.set(
        'routeCodeList',
        this.get('routeCodeList').delete(index)
      ).set(
        'selectedDisplayRouteMainViewer',
        new selectedDisplayRouteMainViewerRecord()
      );
    } else {
      return this.set('routeCodeList', this.get('routeCodeList').delete(index));
    }
  }

  setSelectRouteCodeDisplayMainViewer(selectRouteCodeDisplayMainViewer) {
    console.log(selectRouteCodeDisplayMainViewer);
    return this.set(
      'selectedDisplayRouteMainViewer',
      new selectedDisplayRouteMainViewerRecord()
        .set('type', 'routeCode')
        .set(
          'selectRoute',
          new RouteCodeRecord(selectRouteCodeDisplayMainViewer)
        )
        .set('routeList', this.get('routeCodeList'))
    );
  }

  addRouteCodeByText(textRouteCode) {
    const routeCodeRecord = new RouteCodeRecord()
      .set('startPoint', textRouteCode.startPoint)
      .set('laneList', textRouteCode.laneList)
      .set('endPoint', textRouteCode.endPoint)
      .set('isBack', textRouteCode.isBack)
      .set('routeCode', textRouteCode.routeCode);
    const routeCodeList = this.get('routeCodeList').push(routeCodeRecord);
    return this.set('routeCodeList', routeCodeList).set(
      'isAddRouteByTextModalOpen',
      false
    );
  }

  //Schedule List
  deleteLatestScheduleFromScheduleList() {
    const scheduleList = this.get('scheduleList');
    if (scheduleList.size > 0) {
      const lastSchedule = scheduleList.get(-1).toJS();
      const selectedDisplayRouteMainViewer = this.get(
        'selectedDisplayRouteMainViewer'
      ).toJS().selectRoute;
      if (scheduleList.size === 1) {
        if (
          lastSchedule.routeCode === selectedDisplayRouteMainViewer.routeCode
        ) {
          return this.set('scheduleList', scheduleList.pop())
            .set('lastRoute', new RouteCodeRecord())
            .set('activeStepScheduleEditor', 'selectRouteCode')
            .set('currentRouteCodeSchedule', new RouteCodeRecord())
            .set('checkedSendEngage', false)
            .set('waitTime', 0)
            .set('currentEditChangeRouteList', List())
            .set('isAddScheduleModalOpen', false)
            .set(
              'selectedDisplayRouteMainViewer',
              new selectedDisplayRouteMainViewerRecord()
            );
        } else {
          return this.set('scheduleList', scheduleList.pop())
            .set('lastRoute', new RouteCodeRecord())
            .set('activeStepScheduleEditor', 'selectRouteCode')
            .set('currentRouteCodeSchedule', new RouteCodeRecord())
            .set('checkedSendEngage', false)
            .set('waitTime', 0)
            .set('currentEditChangeRouteList', List())
            .set('isAddScheduleModalOpen', false);
        }
      } else {
        const secondLatestSchedule = scheduleList.get(-2);
        const lastRoute = new RouteCodeRecord()
          .set('routeCode', secondLatestSchedule.routeCode)
          .set('startPoint', secondLatestSchedule.startPoint)
          .set('laneList', secondLatestSchedule.laneList)
          .set('endPoint', secondLatestSchedule.endPoint)
          .set('isBack', secondLatestSchedule.isBack);
        if (
          lastSchedule.routeCode === selectedDisplayRouteMainViewer.routeCode
        ) {
          return this.set('scheduleList', scheduleList.pop())
            .set('lastRoute', lastRoute)
            .set('activeStepScheduleEditor', 'selectRouteCode')
            .set('currentRouteCodeSchedule', new RouteCodeRecord())
            .set('checkedSendEngage', false)
            .set('waitTime', 0)
            .set('currentEditChangeRouteList', List())
            .set('isAddScheduleModalOpen', false)
            .set(
              'selectedDisplayRouteMainViewer',
              new selectedDisplayRouteMainViewerRecord()
            );
        } else {
          return this.set('scheduleList', scheduleList.pop())
            .set('lastRoute', lastRoute)
            .set('activeStepScheduleEditor', 'selectRouteCode')
            .set('currentRouteCodeSchedule', new RouteCodeRecord())
            .set('checkedSendEngage', false)
            .set('waitTime', 0)
            .set('currentEditChangeRouteList', List())
            .set('isAddScheduleModalOpen', false);
        }
      }
    }
    return this;
  }

  openAddScheduleModalAndEditSchedule() {
    const scheduleList = this.get('scheduleList');

    if (scheduleList.size === 1) {
      const latestSchedule = scheduleList.get(-1);
      const currentRoute = new RouteCodeRecord()
        .set('routeCode', latestSchedule.routeCode)
        .set('startPoint', latestSchedule.startPoint)
        .set('laneList', latestSchedule.laneList)
        .set('endPoint', latestSchedule.endPoint)
        .set('isBack', latestSchedule.isBack);
      return this.set('isAddScheduleModalOpen', true)
        .set('activeStepScheduleEditor', 'selectRouteCode')
        .set('selectableRouteCodeList', 'selectRouteCode')
        .set('currentRouteCodeSchedule', currentRoute)
        .set('checkedSendEngage', latestSchedule.checkedSendEngage)
        .set('waitTime', latestSchedule.waitTime)
        .set('currentEditChangeRouteList', List())
        .set('selectableRouteCodeList', this.get('routeCodeList'))
        .set('lastRoute', new RouteCodeRecord())
        .set('scheduleList', scheduleList.pop());
    } else if (scheduleList.size > 1) {
      const latestSchedule = scheduleList.get(-1);
      const secondLatestSchedule = scheduleList.get(-2);

      const currentRoute = new RouteCodeRecord()
        .set('routeCode', latestSchedule.routeCode)
        .set('startPoint', latestSchedule.startPoint)
        .set('laneList', latestSchedule.laneList)
        .set('endPoint', latestSchedule.endPoint)
        .set('isBack', latestSchedule.isBack);

      const lastRoute = new RouteCodeRecord()
        .set('routeCode', secondLatestSchedule.routeCode)
        .set('startPoint', secondLatestSchedule.startPoint)
        .set('laneList', secondLatestSchedule.laneList)
        .set('endPoint', secondLatestSchedule.endPoint)
        .set('isBack', secondLatestSchedule.isBack);

      const RouteCodeList = this.get('routeCodeList').toJS();
      const selectableRouteCodeList = [];
      for (const routeCode of RouteCodeList) {
        const endPoint = lastRoute.endPoint;
        const routeStartPoint = routeCode.startPoint;
        if (endPoint === routeStartPoint) {
          selectableRouteCodeList.push(routeCode);
        }
      }

      return this.set('isAddScheduleModalOpen', true)
        .set('activeStepScheduleEditor', 'selectRouteCode')
        .set('selectableRouteCodeList', 'selectRouteCode')
        .set('currentRouteCodeSchedule', currentRoute)
        .set('checkedSendEngage', latestSchedule.checkedSendEngage)
        .set('waitTime', latestSchedule.waitTime)
        .set('currentEditChangeRouteList', List())
        .set('selectableRouteCodeList', List(selectableRouteCodeList))
        .set('lastRoute', lastRoute)
        .set('scheduleList', scheduleList.pop());
    }

    return this;
  }

  setSelectScheduleDisplayMainViewer(selectScheduleDisplayMainViewer) {
    return this.set(
      'selectedDisplayRouteMainViewer',
      new selectedDisplayRouteMainViewerRecord()
        .set('type', 'schedule')
        .set(
          'selectRoute',
          new RouteCodeRecord(selectScheduleDisplayMainViewer)
        )
        .set('routeList', this.get('scheduleList'))
    );
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

  setIsAddRouteByTextModalOpen(isAddRouteByTextModalOpen) {
    return this.set('isAddRouteByTextModalOpen', isAddRouteByTextModalOpen);
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
      const selectableRouteCodeList = [];
      for (const routeCode of RouteCodeList) {
        const endPoint = lastRoute.endPoint;
        const routeStartPoint = routeCode.startPoint;
        if (endPoint === routeStartPoint) {
          selectableRouteCodeList.push(routeCode);
        }
      }
      return this.set('isAddScheduleModalOpen', isAddScheduleModalOpen).set(
        'selectableRouteCodeList',
        List(selectableRouteCodeList)
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

  getSelectedDisplayRouteMainViewer() {
    return this.get('selectedDisplayRouteMainViewer').toJS();
  }

  getSelectScheduleDisplayMainViewer() {
    return this.get('selectScheduleDisplayMainViewer').toJS();
  }

  //Schedule List
  getScheduleList() {
    return this.get('scheduleList').toJS();
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

  getIsAddRouteByTextModalOpen() {
    return this.get('isAddRouteByTextModalOpen');
  }

  getIsAddScheduleModalOpen() {
    return this.get('isAddScheduleModalOpen');
  }
}
