import {REDUX} from '../../constants/Constant'
import {RouteCodeEditor} from '../../model/Redux/Page/RouteCodeEditor';

export function routeCodeEditorReducer(state = new RouteCodeEditor(), action) {
  if (action.type === REDUX.ACTION_TYPE.SET_HEIGHT_AND_WIDTH_ROUTE_CODE_EDITOR) {
    return state.setHeightAndWidth(action.height, action.width)
  } else if (action.type === REDUX.ACTION_TYPE.SET_ACTIVE_STEP_ROUTE_CODE_EDITOR) {
    return state.setActiveStep(action.activeStep)
  } else if (action.type === REDUX.ACTION_TYPE.SET_IS_BACK_ROUTE_CODE_EDITOR) {
    return state.setIsBack(action.isBack)
  } else if (action.type === REDUX.ACTION_TYPE.REFLECT_MAP_DATA_ROUTE_CODE_EDITOR) {
    return state.setMapData(action.pcd, action.waypoint, action.lane)
  } else if (action.type === REDUX.ACTION_TYPE.SET_START_POINT_ROUTE_CODE_EDITOR) {
    return state.setStartPoint(action.startPoint)
  } else if (action.type === REDUX.ACTION_TYPE.SET_LANE_LIST_ROUTE_CODE_EDITOR) {
    return state.setLaneList(action.laneList)
  } else if (action.type === REDUX.ACTION_TYPE.SET_END_POINT_ROUTE_CODE_EDITOR) {
    return state.setEndPoint(action.endPoint)
  } else if (action.type === REDUX.ACTION_TYPE.SET_START_POINT_AND_LANE_LIST_ROUTE_CODE_EDITOR) {
    return state.setStartPointAndLaneList(action.startPoint, action.laneList)
  } else if (action.type === REDUX.ACTION_TYPE.RESET_ROUTE_CODE_ROUTE_CODE_EDITOR) {
    return state.resetRouteCode()
  } else if (action.type === REDUX.ACTION_TYPE.CLEAR_ROUTE_CODE_DATA_ROUTE_CODE_EDITOR) {
    return state.clearRouteCodeData()
  }

  return state
}

