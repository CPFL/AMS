import {REDUX} from '../../constants/Constant'
import MapPage from '../../model/Redux/Page/MapPage';

export function mapPageReducer(state = new MapPage(), action) {
    if (action.type === REDUX.ACTION_TYPE.SET_IS_SHOW_2D_MAP) {
        return state.setIsShow2DMap(action.isShow2DMap)
    } else if (action.type === REDUX.ACTION_TYPE.SET_HEIGHT_AND_WIDTH_MAP) {
        return state.setHeightAndWidth(action.height, action.width)
    } else if (action.type === REDUX.ACTION_TYPE.SET_IS_SHOW_VEHICLE_LIST_MODAL) {
        return state.setIsShowVehicleListModal(action.isShowVehicleListModal)
    } else if (action.type === REDUX.ACTION_TYPE.SET_SEARCH_KEY_OF_VEHICLE_LIST) {
        return state.setSearchKeyOfVehicleList(action.searchKeyOfVehicleList)
    }
    return state
}

