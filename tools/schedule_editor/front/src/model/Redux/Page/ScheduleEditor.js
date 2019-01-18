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
  isBack: false,
  waypointList: List()
});

const ChangeRouteRecord = new Record({
  routeCode: new RouteCodeRecord(),
  decisionSectionEndPoint: ''
});

const ScheduleRecord = new Record({
  routeCode: '',
  startPoint: '',
  laneList: [],
  endPoint: '',
  isBack: false,
  checkedSendEngage: false,
  waitTime: 0,
  changeRouteList: List(),
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
  routeCodeAfterChangeRoute: null,
  decisionSectionRouteCode: null,
  selectableChangeRouteList: null,
  selectableDecisionSectionEndPointList: null,

  //route code list
  routeCodeList: null,
  selectedDisplayRouteMainViewer: null,

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
      routeCodeAfterChangeRoute: new RouteCodeRecord(),
      decisionSectionRouteCode: new RouteCodeRecord(),
      selectableChangeRouteList: List(),
      selectableDecisionSectionEndPointList: List(),

      //route code list
      routeCodeList: List(),
      selectedDisplayRouteMainViewer: new selectedDisplayRouteMainViewerRecord(),

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

  getRouteCodeWaypointList(startPoint, laneList, endPoint) {
    let selectedRouteCodeWaypoints = [];
    const allLane = this.get('lane').lanes;
    const startLaneWaypoints = allLane[laneList[0]].waypointIDs;
    const startPointIndex = startLaneWaypoints.indexOf(startPoint);
    const endLaneWaypoints = allLane[laneList[laneList.length - 1]].waypointIDs;
    const endPointIndex =
      endLaneWaypoints.length - endLaneWaypoints.indexOf(endPoint);
    for (const lane of laneList) {
      selectedRouteCodeWaypoints = selectedRouteCodeWaypoints.concat(
        allLane[lane].waypointIDs
      );
    }
    selectedRouteCodeWaypoints.splice(0, startPointIndex);
    selectedRouteCodeWaypoints.splice(
      selectedRouteCodeWaypoints.length - endPointIndex + 1,
      selectedRouteCodeWaypoints.length
    );
    return selectedRouteCodeWaypoints;
  }

  saveRouteCode(routeCode) {
    const startPoint = this.get('startPoint');
    const laneList = this.get('laneList').toJS();
    const endPoint = this.get('endPoint');
    const isBack = this.get('isBack');
    const waypointList = this.getRouteCodeWaypointList(
      startPoint,
      laneList,
      endPoint
    );

    const routeCodeRecord = new RouteCodeRecord()
      .set('startPoint', startPoint)
      .set('laneList', laneList)
      .set('endPoint', endPoint)
      .set('isBack', isBack)
      .set('routeCode', routeCode)
      .set('waypointList', waypointList);
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
    const startPoint = this.get('startPoint');
    const laneList = this.get('laneList').toJS();
    const endPoint = this.get('endPoint');
    const isBack = this.get('isBack');
    const waypointList = this.getRouteCodeWaypointList(
      startPoint,
      laneList,
      endPoint
    );

    const routeCodeRecord = new RouteCodeRecord()
      .set('startPoint', startPoint)
      .set('laneList', laneList)
      .set('endPoint', endPoint)
      .set('isBack', isBack)
      .set('routeCode', routeCode)
      .set('waypointList', waypointList);

    const routeCodeList = this.get('routeCodeList').push(routeCodeRecord);
    return this.set('startPoint', this.get('endPoint'))
      .set('laneList', List([this.get('laneList').get(-1)]))
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
    if (activeStepScheduleEditor === 'changeRouteEditor') {
      const selectedRouteCodeWaypoints = this.get(
        'currentRouteCodeSchedule'
      ).toJS().waypointList;
      if (selectedRouteCodeWaypoints.length > 10) {
        selectedRouteCodeWaypoints.splice(0, 5);
        selectedRouteCodeWaypoints.splice(
          selectedRouteCodeWaypoints.length - 5,
          selectedRouteCodeWaypoints.length
        );

        const routeCodeList = this.get('routeCodeList');
        const selectableChangeRouteList = [];

        for (const routeCode of routeCodeList) {
          for (const waypoint of selectedRouteCodeWaypoints) {
            if (routeCode.startPoint === waypoint) {
              selectableChangeRouteList.push(routeCode);
            }
          }
        }
        return this.set(
          'activeStepScheduleEditor',
          activeStepScheduleEditor
        ).set('selectableChangeRouteList', List(selectableChangeRouteList));
      } else {
        return this;
      }
    } else {
      return this.set('activeStepScheduleEditor', activeStepScheduleEditor);
    }
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
  setChangeRouteActiveStepNext() {
    const changeRouteActiveStep = this.get('changeRouteActiveStep') + 1;
    return this.set('changeRouteActiveStep', changeRouteActiveStep);
  }

  setChangeRouteActiveStepPrevious() {
    let changeRouteActiveStep = this.get('changeRouteActiveStep') - 1;
    console.log(changeRouteActiveStep);
    if (changeRouteActiveStep < 0) {
      changeRouteActiveStep = 0;
    }
    return this.set('changeRouteActiveStep', changeRouteActiveStep);
  }

  setChangeRouteActiveStepReset() {
    return this.set('changeRouteActiveStep', 0);
  }

  setRouteCodeAfterChangeRoute(routeCodeAfterChangeRoute) {
    const selectableDecisionSectionEndPointList = [];
    const routeCodeAfterChangeRouteWaypointList =
      routeCodeAfterChangeRoute.waypointList;
    const scheduleRouteCodeWaypointList = this.get(
      'currentRouteCodeSchedule'
    ).toJS().waypointList;
    for (const waypoint of routeCodeAfterChangeRouteWaypointList) {
      if (scheduleRouteCodeWaypointList.includes(waypoint)) {
        selectableDecisionSectionEndPointList.push(waypoint);
      } else {
        break;
      }
    }
    return this.set(
      'routeCodeAfterChangeRoute',
      new RouteCodeRecord(routeCodeAfterChangeRoute)
    ).set(
      'selectableDecisionSectionEndPointList',
      selectableDecisionSectionEndPointList
    );
  }

  setDecisionSectionEndPoint(decisionSectionEndPoint) {
    const routeCodeAfterChangeRoute = this.get('routeCodeAfterChangeRoute');
    const decisionSectionStartPoint = routeCodeAfterChangeRoute.startPoint;
    const allLanes = this.get('lane').lanes;
    const decisionSectionLaneList = [];
    for (const laneID of routeCodeAfterChangeRoute.laneList) {
      decisionSectionLaneList.push(laneID);
      if (allLanes[laneID].waypointIDs.includes(decisionSectionEndPoint)) {
        break;
      }
    }
    const decisionSectionIsBack = routeCodeAfterChangeRoute.isBack;

    let lastPoint;
    let laneString = '';
    for (const lane of decisionSectionLaneList) {
      const points = lane.split('_');

      const [arrow, startIndex, endIndex] = decisionSectionIsBack
        ? ['<', 1, 0]
        : ['>', 0, 1];

      laneString += points[startIndex] + arrow;
      lastPoint = points[endIndex];
    }
    laneString += lastPoint;

    const decisionSectionRouteCode =
      decisionSectionStartPoint +
      ':' +
      laneString +
      ':' +
      decisionSectionEndPoint;

    const decisionSectionWaypointList = this.getRouteCodeWaypointList(
      decisionSectionStartPoint,
      decisionSectionLaneList,
      decisionSectionEndPoint
    );
    const decisionSectionRouteCodeRecord = new RouteCodeRecord()
      .set('startPoint', decisionSectionStartPoint)
      .set('laneList', decisionSectionLaneList)
      .set('endPoint', decisionSectionEndPoint)
      .set('isBack', decisionSectionIsBack)
      .set('routeCode', decisionSectionRouteCode)
      .set('waypointList', decisionSectionWaypointList);
    return this.set('decisionSectionRouteCode', decisionSectionRouteCodeRecord);
  }

  //Route Code List
  addContinueRoute(previousRoute) {
    const startPoint = previousRoute.endPoint;
    const laneList = previousRoute.laneList;
    return this.set('isAddRouteModalOpen', true)
      .set('startPoint', startPoint)
      .set('laneList', List([laneList.pop()]))
      .set('endPoint', '')
      .set('isBack', false)
      .set('activeStep', steps.advanceOrBack.id);
  }

  deleteRouteCodeFromRouteCodeListByIndex(index) {
    const deleteRecord = this.get('routeCodeList')
      .get(index)
      .toJS();
    const selectedDisplayRouteMainViewer = this.get(
      'selectedDisplayRouteMainViewer'
    ).toJS().selectRoute;

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

  addRouteCodeByText(textRouteCodeList) {
    let routeCodeList = this.get('routeCodeList');
    for (const textRouteCode of textRouteCodeList) {
      const startPoint = textRouteCode.startPoint;
      const laneList = textRouteCode.laneList;
      const endPoint = textRouteCode.endPoint;
      const isBack = textRouteCode.isBack;
      const waypointList = this.getRouteCodeWaypointList(
        startPoint,
        laneList,
        endPoint
      );

      const routeCodeRecord = new RouteCodeRecord()
        .set('startPoint', startPoint)
        .set('laneList', laneList)
        .set('endPoint', endPoint)
        .set('isBack', isBack)
        .set('routeCode', textRouteCode.routeCode)
        .set('waypointList', waypointList);

      routeCodeList = routeCodeList.push(routeCodeRecord);
    }

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

  getRouteCodeAfterChangeRoute() {
    return this.get('routeCodeAfterChangeRoute').toJS();
  }

  getSelectableChangeRouteList() {
    return this.get('selectableChangeRouteList').toJS();
  }

  getDecisionSectionRouteCode() {
    return this.get('decisionSectionRouteCode').toJS();
  }

  getSelectableDecisionSectionEndPointList() {
    return this.get('selectableDecisionSectionEndPointList');
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
