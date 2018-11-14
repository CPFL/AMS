import {Record} from 'immutable';

const MapPageRecord = new Record({
  isShow2DMap: true,
  height: 0,
  width: 0,
  isShowVehicleListModal: false,
  searchKeyOfVehicleList: ""
});

export default class MapPage extends MapPageRecord {

  constructor() {
    super({
      isShow2DMap: false,
      height: 1000,
      width: 1000,
      isShowVehicleListModal: false,
      searchKeyOfVehicleList: ""
    });
  }

  setIsShow2DMap(isShow2DMap) {
    return this.set('isShow2DMap', isShow2DMap)
  }

  setHeightAndWidth(height, width) {
    return this.set('height', height).set('width', width)
  }

  setIsShowVehicleListModal(isShowVehicleListModal) {
    return this.set('isShowVehicleListModal', isShowVehicleListModal)
  }

  setSearchKeyOfVehicleList(searchKeyOfVehicleList) {
    return this.set('searchKeyOfVehicleList', searchKeyOfVehicleList)
  }

  getIsShow2DMap() {
    return this.get('isShow2DMap')
  }

  getHeight() {
    return this.get('height')
  }

  getWidth() {
    return this.get('width')
  }

  getIsShowVehicleListModal() {
    return this.get('isShowVehicleListModal')
  }

  getSearchKeyOfVehicleList() {
    return this.get('searchKeyOfVehicleList')
  }

}