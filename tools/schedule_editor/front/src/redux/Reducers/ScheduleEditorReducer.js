import {REDUX} from '../../constants/Constant'
import {ScheduleEditor} from '../../model/Redux/Page/ScheduleEditor';

export function scheduleEditorReducer(state = new ScheduleEditor(), action) {
  if (action.type === REDUX.ACTION_TYPE.SET_ACTIVE_STEP_SCHEDULE_EDITOR) {
    return state.setActiveStep(action.activeStep)
  } else if (action.type === REDUX.ACTION_TYPE.BACK_STEP_SCHEDULE_EDITOR) {
    return state.backStep(action.activeStep)
  } else if (action.type === REDUX.ACTION_TYPE.SET_IS_BACK_SCHEDULE_EDITOR) {
    return state.setIsBack(action.isBack)
  } else if (action.type === REDUX.ACTION_TYPE.REFLECT_MAP_DATA_SCHEDULE_EDITOR) {
    return state.setMapData(action.pcd, action.waypoint, action.lane)
  } else if (action.type === REDUX.ACTION_TYPE.SET_START_POINT_SCHEDULE_EDITOR) {
    return state.setStartPoint(action.startPoint)
  } else if (action.type === REDUX.ACTION_TYPE.SET_LANE_LIST_SCHEDULE_EDITOR) {
    return state.setLaneList(action.laneList)
  } else if (action.type === REDUX.ACTION_TYPE.SET_END_POINT_SCHEDULE_EDITOR) {
    return state.setEndPoint(action.endPoint)
  } else if (action.type === REDUX.ACTION_TYPE.SET_START_POINT_AND_LANE_LIST_SCHEDULE_EDITOR) {
    return state.setStartPointAndLaneList(action.startPoint, action.laneList)
  } else if (action.type === REDUX.ACTION_TYPE.SAVE_ROUTE_CODE_SCHEDULE_EDITOR) {
    return state.saveRouteCode(action.routeCode)
  } else if (action.type === REDUX.ACTION_TYPE.SAVE_AND_ANOTHER_SELECT_ROUTE_CODE_SCHEDULE_EDITOR) {
    return state.saveAndAnotherSelectRouteCode(action.routeCode)
  } else if (action.type === REDUX.ACTION_TYPE.RESET_ROUTE_CODE_SCHEDULE_EDITOR) {
    return state.resetRouteCode()
  } else if (action.type === REDUX.ACTION_TYPE.CLEAR_ROUTE_CODE_DATA_SCHEDULE_EDITOR) {
    return state.clearRouteCodeData()
  } else if (action.type === REDUX.ACTION_TYPE.SET_IS_IMPORT_DATA_MODAL_HEADER) {
    return state.setIsImportDataModalOpen(action.isImportDataModalOpen)
  } else if (action.type === REDUX.ACTION_TYPE.SET_IS_ADD_ROUTE_MODAL_SCHEDULE_EDITOR) {
    return state.setIsAddRouteModalOpen(action.isAddRouteModalOpen)
  }

  return state
}

