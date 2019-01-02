import { REDUX } from '../../constants/Constant';

//Route Code Editor
export function setActiveStep(activeStep) {
  return {
    type: REDUX.ACTION_TYPE.SET_ACTIVE_STEP_ROUTE_CODE_EDITOR,
    activeStep: activeStep
  };
}

export function backStep(activeStep) {
  return {
    type: REDUX.ACTION_TYPE.BACK_STEP_ROUTE_CODE_EDITOR,
    activeStep: activeStep
  };
}

export function setIsBack(isBack) {
  return {
    type: REDUX.ACTION_TYPE.SET_IS_BACK_ROUTE_CODE_EDITOR,
    isBack: isBack
  };
}

export function setMapData(pcd, waypoint, lane) {
  return {
    type: REDUX.ACTION_TYPE.SET_MAP_DATA_SCHEDULE_EDITOR,
    pcd: pcd,
    waypoint: waypoint,
    lane: lane
  };
}

export function setStartPoint(startPoint) {
  return {
    type: REDUX.ACTION_TYPE.SET_START_POINT_ROUTE_CODE_EDITOR,
    startPoint: startPoint
  };
}

export function setLaneList(laneList) {
  return {
    type: REDUX.ACTION_TYPE.SET_LANE_LIST_ROUTE_CODE_EDITOR,
    laneList: laneList
  };
}

export function setEndPoint(endPoint) {
  return {
    type: REDUX.ACTION_TYPE.SET_END_POINT_ROUTE_CODE_EDITOR,
    endPoint: endPoint
  };
}

export function setStartPointAndLaneList(startPoint, laneList) {
  return {
    type: REDUX.ACTION_TYPE.SET_START_POINT_AND_LANE_LIST_SCHEDULE_EDITOR,
    startPoint: startPoint,
    laneList: laneList
  };
}

export function saveRouteCode(routeCode) {
  return {
    type: REDUX.ACTION_TYPE.SAVE_ROUTE_CODE,
    routeCode: routeCode
  };
}

export function saveAndAnotherSelectRouteCode(routeCode) {
  return {
    type: REDUX.ACTION_TYPE.SAVE_AND_ANOTHER_SELECT_ROUTE_CODE,
    routeCode: routeCode
  };
}

export function resetRouteCode() {
  return {
    type: REDUX.ACTION_TYPE.RESET_ROUTE_CODE
  };
}

//Schedule Editor

export function setActiveStepScheduleEditor(activeStepScheduleEditor) {
  return {
    type: REDUX.ACTION_TYPE.SET_ACTIVE_STEP_SCHEDULE_EDITOR,
    activeStepScheduleEditor: activeStepScheduleEditor
  };
}

export function setCurrentRouteCodeSchedule(currentRouteCodeSchedule) {
  return {
    type: REDUX.ACTION_TYPE.SER_CURRENT_ROUTE_CODE_SCHEDULE,
    currentRouteCodeSchedule: currentRouteCodeSchedule
  };
}

export function setCheckedSendEngage(checkedSendEngage) {
  return {
    type: REDUX.ACTION_TYPE.SET_CHECKED_SEND_ENGAGE,
    checkedSendEngage: checkedSendEngage
  };
}

export function setWaitTime(waitTime) {
  return {
    type: REDUX.ACTION_TYPE.SET_WAIT_TIME,
    waitTime: waitTime
  };
}

export function setCurrentEditChangeRouteList(currentEditChangeRouteList) {
  return {
    type: REDUX.ACTION_TYPE.SET_CURRENT_EDIT_CHANGE_ROUTE_LIST,
    currentEditChangeRouteList: currentEditChangeRouteList
  };
}

export function saveSchedule() {
  return {
    type: REDUX.ACTION_TYPE.SAVE_SCHEDULE
  };
}

//Change Route Editor

export function setChangeRouteActiveStepNext(changeRouteActiveStep) {
  return {
    type: REDUX.ACTION_TYPE.SET_CHANGE_ROUTE_ACTIVE_STEP_NEXT,
    changeRouteActiveStep: changeRouteActiveStep
  };
}

export function setChangeRouteActiveStepPrevious(changeRouteActiveStep) {
  return {
    type: REDUX.ACTION_TYPE.SET_CHANGE_ROUTE_ACTIVE_STEP_PREVIOUS,
    changeRouteActiveStep: changeRouteActiveStep
  };
}

export function setChangeRouteActiveStepReset(changeRouteActiveStep) {
  return {
    type: REDUX.ACTION_TYPE.SET_CHANGE_ROUTE_ACTIVE_STEP_RESET,
    changeRouteActiveStep: changeRouteActiveStep
  };
}

//Route Code List
export function addContinueRoute(previousRoute) {
  return {
    type: REDUX.ACTION_TYPE.ADD_CONTINUE_ROUTE,
    previousRoute: previousRoute
  };
}

export function deleteRouteCodeFromRouteCodeListByIndex(index) {
  return {
    type: REDUX.ACTION_TYPE.DELETE_ROUTE_CODE_FROM_ROUTE_CODE_LIST_BY_INDEX,
    index: index
  };
}

export function addRouteCodeByText(textRouteCode) {
  return {
    type: REDUX.ACTION_TYPE.ADD_ROUTE_CODE_BY_TEXT,
    textRouteCode: textRouteCode
  };
}

export function setSelectRouteCodeDisplayMainViewer(
  selectRouteCodeDisplayMainViewer
) {
  return {
    type: REDUX.ACTION_TYPE.SET_SELECT_ROUTE_CODE_DISPLAY_MAIN_VIEWER,
    selectRouteCodeDisplayMainViewer: selectRouteCodeDisplayMainViewer
  };
}

//Schedule List
export function deleteLatestScheduleFromScheduleList() {
  return {
    type: REDUX.ACTION_TYPE.DELETE_LATEST_SCHEDULE_FROM_SCHEDULE_LIST
  };
}

export function openAddScheduleModalAndEditSchedule() {
  return {
    type: REDUX.ACTION_TYPE.OPEN_ADD_SCHEDULE_MODAL_AND_EDIT_SCHEDULE
  };
}

export function setSelectScheduleDisplayMainViewer(
  selectScheduleDisplayMainViewer
) {
  return {
    type: REDUX.ACTION_TYPE.SET_SELECT_SCHEDULE_DISPLAY_MAIN_VIEWER,
    selectScheduleDisplayMainViewer: selectScheduleDisplayMainViewer
  };
}

//Modal
export function setIsImportDataModalOpen(isImportDataModalOpen) {
  return {
    type: REDUX.ACTION_TYPE.SET_IS_IMPORT_DATA_MODAL_OPEN,
    isImportDataModalOpen: isImportDataModalOpen
  };
}

export function setIsAddRouteModalOpen(isAddRouteModalOpen) {
  return {
    type: REDUX.ACTION_TYPE.SET_IS_ADD_ROUTE_MODAL_OPEN,
    isAddRouteModalOpen: isAddRouteModalOpen
  };
}

export function setIsAddRouteByTextModalOpen(isAddRouteByTextModalOpen) {
  return {
    type: REDUX.ACTION_TYPE.SET_IS_ADD_ROUTE_BY_TEXT_MODAL_OPEN,
    isAddRouteByTextModalOpen: isAddRouteByTextModalOpen
  };
}

export function setIsAddScheduleModalOpen(isAddScheduleModalOpen) {
  return {
    type: REDUX.ACTION_TYPE.SET_IS_ADD_SCHEDULE_MODAL_OPEN,
    isAddScheduleModalOpen: isAddScheduleModalOpen
  };
}
