import {createSelectorCreator, defaultMemoize} from 'reselect';
import isEqual from 'lodash.isequal';

const mapData = (state) => state.routeCodeEditor.getMapData();
const startPoint = (state) => state.routeCodeEditor.getStartPoint();
const laneList = (state) => state.routeCodeEditor.getLaneList();
const endPoint = (state) => state.routeCodeEditor.getEndPoint();


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

export const routeCodeSelector = createDeepEqualSelector(
  [startPoint, laneList, endPoint],
  (startPoint, laneList, endPoint) => {
    return {
      startPoint: startPoint,
      laneList: laneList,
      endPoint: endPoint
    }
  }
);

