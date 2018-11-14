import {REDUX} from '../../constants/Constant'
import {RouteCodeEditor} from '../../model/Redux/Page/RouteCodeEditor';

export function routeCodeEditorReducer(state = new RouteCodeEditor(), action) {
    if (action.type === REDUX.ACTION_TYPE.SET_HEIGHT_AND_WIDTH_ROUTE_CODE_EDITOR) {
        return state.setHeightAndWidth(action.height, action.width)
    }else if (action.type === REDUX.ACTION_TYPE.SET_ACTIVE_STEP_ROUTE_CODE_EDITOR) {
      return state.setActiveStep(action.activeStep)
    }else if (action.type === REDUX.ACTION_TYPE.SET_IS_BACK_ROUTE_CODE_EDITOR) {
      return state.setIsBack(action.isBack)
    }

    return state
}

