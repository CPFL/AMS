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

export function setIsBack(isBack) {
  return {
    type: REDUX.ACTION_TYPE.SET_IS_BACK_ROUTE_CODE_EDITOR,
    isBack: isBack
  }
}
