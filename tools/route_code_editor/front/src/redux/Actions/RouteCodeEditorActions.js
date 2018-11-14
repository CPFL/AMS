import {REDUX} from '../../constants/Constant'

export function setIsShow2DMap(isShow2DMap) {
    return {
        type: REDUX.ACTION_TYPE.SET_IS_SHOW_2D_MAP,
        isShow2DMap: isShow2DMap
    }
}

export function setHeightAndWidth(height, width) {
    return {
        type: REDUX.ACTION_TYPE.SET_HEIGHT_AND_WIDTH_MAP,
        height: height,
        width: width
    }
}

export function setIsShowVehicleListModal(isShowVehicleListModal) {
    return {
        type: REDUX.ACTION_TYPE.SET_IS_SHOW_VEHICLE_LIST_MODAL,
        isShowVehicleListModal: isShowVehicleListModal
    }
}

export function setSearchKeyOfVehicleList(searchKeyOfVehicleList) {
    return {
        type: REDUX.ACTION_TYPE.SET_SEARCH_KEY_OF_VEHICLE_LIST,
        searchKeyOfVehicleList: searchKeyOfVehicleList
    }
}
