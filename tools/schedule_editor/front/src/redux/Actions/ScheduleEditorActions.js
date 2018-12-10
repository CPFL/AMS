import {REDUX} from '../../constants/Constant'

export function setActiveStep(activeStep) {
  return {
    type: REDUX.ACTION_TYPE.SET_ACTIVE_STEP_SCHEDULE_EDITOR,
    activeStep: activeStep
  }
}

export function backStep(activeStep) {
  return {
    type: REDUX.ACTION_TYPE.BACK_STEP_SCHEDULE_EDITOR,
    activeStep: activeStep
  }
}

export function setIsBack(isBack) {
  return {
    type: REDUX.ACTION_TYPE.SET_IS_BACK_SCHEDULE_EDITOR,
    isBack: isBack
  }
}

export function setMapData(pcd, waypoint, lane) {
  return {
    type: REDUX.ACTION_TYPE.REFLECT_MAP_DATA_SCHEDULE_EDITOR,
    pcd: pcd,
    waypoint: waypoint,
    lane: lane
  }
}

export function setStartPoint(startPoint) {
  return {
    type: REDUX.ACTION_TYPE.SET_START_POINT_SCHEDULE_EDITOR,
    startPoint: startPoint
  }
}

export function setLaneList(laneList) {
  return {
    type: REDUX.ACTION_TYPE.SET_LANE_LIST_SCHEDULE_EDITOR,
    laneList: laneList
  }
}

export function setEndPoint(endPoint) {
  return {
    type: REDUX.ACTION_TYPE.SET_END_POINT_SCHEDULE_EDITOR,
    endPoint: endPoint
  }
}

export function setStartPointAndLaneList(startPoint, laneList) {
  return {
    type: REDUX.ACTION_TYPE.SET_START_POINT_AND_LANE_LIST_SCHEDULE_EDITOR,
    startPoint: startPoint,
    laneList: laneList
  }
}


export function resetRouteCode() {
  return {
    type: REDUX.ACTION_TYPE.RESET_ROUTE_CODE_SCHEDULE_EDITOR
  }
}

export function setIsImportDataModalOpen(isImportDataModalOpen) {
  return {
    type: REDUX.ACTION_TYPE.SET_IS_IMPORT_DATA_MODAL_HEADER,
    isImportDataModalOpen: isImportDataModalOpen
  }
}

