import {createSelectorCreator, defaultMemoize} from 'reselect';
import isEqual from 'lodash.isequal';

const mapData = (state) => state.routeCodeEditor.getMapData();
const getCenterVehicleId = (state) => state.routeCodeEditor.getCenterVehicleId();
const getVehicleStatusList = (state) => state.routeCodeEditor.getVehicleStatusList();
const getLatestVehicleStatus = (state) => state.routeCodeEditor.getLatestVehicleStatus();


const createDeepEqualSelector = createSelectorCreator(
  defaultMemoize,
  isEqual
);

export const mapDataSelector = createDeepEqualSelector(
  [mapData],
  mapData => {
    return mapData
  }
);

export const CenterVehicleIdSelector = createDeepEqualSelector(
  [getCenterVehicleId],
  getCenterVehicleId => {
    return getCenterVehicleId
  }
);

export const VehicleStatusListSelector = createDeepEqualSelector(
  [getVehicleStatusList],
  getVehicleStatusList => {
    return getVehicleStatusList
  }
);

export const LatestVehicleStatusSelector = createDeepEqualSelector(
  [getLatestVehicleStatus],
  getLatestVehicleStatus => {
    return getLatestVehicleStatus
  }
);
