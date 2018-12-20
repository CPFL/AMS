import { REDUX } from '../../constants/Constant';
import { ScheduleEditor } from '../../model/Redux/Page/ScheduleEditor';

export function scheduleEditorReducer(state = new ScheduleEditor(), action) {
  //Route Code Editor
  if (action.type === REDUX.ACTION_TYPE.SET_ACTIVE_STEP_ROUTE_CODE_EDITOR) {
    return state.setActiveStep(action.activeStep);
  } else if (action.type === REDUX.ACTION_TYPE.BACK_STEP_ROUTE_CODE_EDITOR) {
    return state.backStep(action.activeStep);
  } else if (action.type === REDUX.ACTION_TYPE.SET_IS_BACK_ROUTE_CODE_EDITOR) {
    return state.setIsBack(action.isBack);
  } else if (action.type === REDUX.ACTION_TYPE.SET_MAP_DATA_SCHEDULE_EDITOR) {
    return state.setMapData(action.pcd, action.waypoint, action.lane);
  } else if (
    action.type === REDUX.ACTION_TYPE.SET_START_POINT_ROUTE_CODE_EDITOR
  ) {
    return state.setStartPoint(action.startPoint);
  } else if (
    action.type === REDUX.ACTION_TYPE.SET_LANE_LIST_ROUTE_CODE_EDITOR
  ) {
    return state.setLaneList(action.laneList);
  } else if (
    action.type === REDUX.ACTION_TYPE.SET_END_POINT_ROUTE_CODE_EDITOR
  ) {
    return state.setEndPoint(action.endPoint);
  } else if (
    action.type ===
    REDUX.ACTION_TYPE.SET_START_POINT_AND_LANE_LIST_SCHEDULE_EDITOR
  ) {
    return state.setStartPointAndLaneList(action.startPoint, action.laneList);
  } else if (action.type === REDUX.ACTION_TYPE.SAVE_ROUTE_CODE) {
    return state.saveRouteCode(action.routeCode);
  } else if (
    action.type === REDUX.ACTION_TYPE.SAVE_AND_ANOTHER_SELECT_ROUTE_CODE
  ) {
    return state.saveAndAnotherSelectRouteCode(action.routeCode);
  } else if (action.type === REDUX.ACTION_TYPE.RESET_ROUTE_CODE) {
    return state.resetRouteCode();
  } else if (action.type === REDUX.ACTION_TYPE.CLEAR_ROUTE_CODE_DATA) {
    return state.clearRouteCodeData();
  }
  //Change Route Editor
  else if (
    action.type === REDUX.ACTION_TYPE.SET_CHANGE_ROUTE_ACTIVE_STEP_NEXT
  ) {
    return state.setChangeRouteActiveStepNext(action.changeRouteActiveStep);
  } else if (
    action.type === REDUX.ACTION_TYPE.SET_CHANGE_ROUTE_ACTIVE_STEP_PREVIOUS
  ) {
    return state.setChangeRouteActiveStepPrevious(action.changeRouteActiveStep);
  } else if (
    action.type === REDUX.ACTION_TYPE.SET_CHANGE_ROUTE_ACTIVE_STEP_RESET
  ) {
    return state.setChangeRouteActiveStepReset(action.changeRouteActiveStep);
  } else if (action.type === REDUX.ACTION_TYPE.SET_CHANGE_ROUTE_START_POINT) {
    return state.setChangeRouteStartPoint(action.startPoint);
  }
  // Schedule Editor
  else if (action.type === REDUX.ACTION_TYPE.SET_ACTIVE_STEP_SCHEDULE_EDITOR) {
    return state.setActiveStepScheduleEditor(action.activeStepScheduleEditor);
  } else if (
    action.type === REDUX.ACTION_TYPE.SER_CURRENT_ROUTE_CODE_SCHEDULE
  ) {
    return state.setCurrentRouteCodeSchedule(action.currentRouteCodeSchedule);
  } else if (action.type === REDUX.ACTION_TYPE.SET_CHECKED_SEND_ENGAGE) {
    return state.setCheckedSendEngage(action.checkedSendEngage);
  } else if (action.type === REDUX.ACTION_TYPE.SET_WAIT_TIME) {
    return state.setWaitTime(action.waitTime);
  } else if (
    action.type === REDUX.ACTION_TYPE.SET_CURRENT_EDIT_CHANGE_ROUTE_LIST
  ) {
    return state.setCurrentEditChangeRouteList(
      action.currentEditChangeRouteList
    );
  } else if (action.type === REDUX.ACTION_TYPE.SAVE_SCHEDULE) {
    return state.saveSchedule();
  }
  // Modal
  else if (action.type === REDUX.ACTION_TYPE.SET_IS_IMPORT_DATA_MODAL_OPEN) {
    return state.setIsImportDataModalOpen(action.isImportDataModalOpen);
  } else if (action.type === REDUX.ACTION_TYPE.SET_IS_ADD_ROUTE_MODAL_OPEN) {
    return state.setIsAddRouteModalOpen(action.isAddRouteModalOpen);
  } else if (action.type === REDUX.ACTION_TYPE.SET_IS_ADD_SCHEDULE_MODAL_OPEN) {
    return state.setIsAddScheduleModalOpen(action.isAddScheduleModalOpen);
  }

  return state;
}
