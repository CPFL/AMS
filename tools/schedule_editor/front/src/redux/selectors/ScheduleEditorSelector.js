import { createSelectorCreator, defaultMemoize } from 'reselect';
import isEqual from 'lodash.isequal';

const mapData = state => state.scheduleEditor.getMapData();
const startPoint = state => state.scheduleEditor.getStartPoint();
const laneList = state => state.scheduleEditor.getLaneList();
const endPoint = state => state.scheduleEditor.getEndPoint();
const selectedDisplayRouteMainViewer = state =>
  state.scheduleEditor.getSelectedDisplayRouteMainViewer();
const selectScheduleDisplayMainViewer = state =>
  state.scheduleEditor.getSelectScheduleDisplayMainViewer();
const currentRouteCodeSchedule = state =>
  state.scheduleEditor.getCurrentRouteCodeSchedule();

const createDeepEqualSelector = createSelectorCreator(defaultMemoize, isEqual);

export const mapDataSelector = createDeepEqualSelector([mapData], mapData => {
  return mapData;
});

export const routeCodeSelector = createDeepEqualSelector(
  [startPoint, laneList, endPoint],
  (startPoint, laneList, endPoint) => {
    return {
      startPoint: startPoint,
      laneList: laneList,
      endPoint: endPoint
    };
  }
);

export const selectedDisplayRouteMainViewerSelector = createDeepEqualSelector(
  [selectedDisplayRouteMainViewer],
  selectedDisplayRouteMainViewer => {
    return selectedDisplayRouteMainViewer;
  }
);

export const selectScheduleDisplayMainViewerSelector = createDeepEqualSelector(
  [selectScheduleDisplayMainViewer],
  selectScheduleDisplayMainViewer => {
    return selectScheduleDisplayMainViewer;
  }
);

export const currentRouteCodeScheduleSelector = createDeepEqualSelector(
  [currentRouteCodeSchedule],
  currentRouteCodeSchedule => {
    return currentRouteCodeSchedule;
  }
);
