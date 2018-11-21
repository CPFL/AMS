import {REDUX} from '../../constants/Constant'

export function setHeightAndWidth(height, width) {
    return {
        type: REDUX.ACTION_TYPE.SET_HEIGHT_AND_WIDTH_ROUTE_CODE_EDITOR,
        height: height,
        width: width
    }
}

export function setActiveStep(activeStep) {
  return {
    type: REDUX.ACTION_TYPE.SET_ACTIVE_STEP_ROUTE_CODE_EDITOR,
    activeStep: activeStep
  }
}

export function backStep(activeStep) {
  return {
    type: REDUX.ACTION_TYPE.SET_ACTIVE_STEP_ROUTE_CODE_EDITOR,
    activeStep: activeStep
  }
}

export function setIsBack(isBack) {
  return {
    type: REDUX.ACTION_TYPE.SET_IS_BACK_ROUTE_CODE_EDITOR,
    isBack: isBack
  }
}

export function setMapData(pcd, waypoint, lane) {
  return {
    type: REDUX.ACTION_TYPE.REFLECT_MAP_DATA_ROUTE_CODE_EDITOR,
    pcd: pcd,
    waypoint: waypoint,
    lane: lane
  }
}

export function setStartPoint(startPoint) {
  return {
    type: REDUX.ACTION_TYPE.SET_START_POINT_ROUTE_CODE_EDITOR,
    startPoint: startPoint
  }
}

export function setLaneList(laneList) {
  return {
    type: REDUX.ACTION_TYPE.SET_LANE_LIST_ROUTE_CODE_EDITOR,
    laneList: laneList
  }
}

export function setEndPoint(endPoint) {
  return {
    type: REDUX.ACTION_TYPE.SET_END_POINT_ROUTE_CODE_EDITOR,
    endPoint: endPoint
  }
}

export function setStartPointAndLaneList(startPoint, laneList) {
  return {
    type: REDUX.ACTION_TYPE.SET_START_POINT_AND_LANE_LIST_ROUTE_CODE_EDITOR,
    startPoint: startPoint,
    laneList: laneList
  }
}


export function resetRouteCode() {
  return {
    type: REDUX.ACTION_TYPE.RESET_ROUTE_CODE_ROUTE_CODE_EDITOR
  }
}
