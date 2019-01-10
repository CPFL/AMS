import { createSelectorCreator, defaultMemoize } from 'reselect';
import isEqual from 'lodash.isequal';

const mapData = state => state.scheduleEditor.getMapData();

//Route Code Editor
const startPoint = state => state.scheduleEditor.getStartPoint();
const laneList = state => state.scheduleEditor.getLaneList();
const endPoint = state => state.scheduleEditor.getEndPoint();

//Main Viewer
const selectedDisplayRouteMainViewer = state =>
  state.scheduleEditor.getSelectedDisplayRouteMainViewer();

//Change Route
const changeRouteActiveStep = state =>
  state.scheduleEditor.getChangeRouteActiveStep();
const currentRouteCodeSchedule = state =>
  state.scheduleEditor.getCurrentRouteCodeSchedule();
const routeCodeAfterChangeRoute = state =>
  state.scheduleEditor.getRouteCodeAfterChangeRoute();
const selectableDecisionSectionEndPointList = state =>
  state.scheduleEditor.getSelectableDecisionSectionEndPointList();
const decisionSectionRouteCode = state =>
  state.scheduleEditor.getDecisionSectionRouteCode();

const createDeepEqualSelector = createSelectorCreator(defaultMemoize, isEqual);

export const mapDataSelector = createDeepEqualSelector([mapData], mapData => {
  return mapData;
});

//Route Code Editor

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

//Main Viewer

export const selectedDisplayRouteMainViewerSelector = createDeepEqualSelector(
  [selectedDisplayRouteMainViewer],
  selectedDisplayRouteMainViewer => {
    return selectedDisplayRouteMainViewer;
  }
);

//Change Route

export const currentRouteCodeScheduleSelector = createDeepEqualSelector(
  [currentRouteCodeSchedule],
  currentRouteCodeSchedule => {
    return currentRouteCodeSchedule;
  }
);

export const changeRouteActiveStepSelector = createDeepEqualSelector(
  [changeRouteActiveStep],
  changeRouteActiveStep => {
    return changeRouteActiveStep;
  }
);

export const routeCodeAfterChangeRouteSelector = createDeepEqualSelector(
  [routeCodeAfterChangeRoute],
  routeCodeAfterChangeRoute => {
    return routeCodeAfterChangeRoute;
  }
);

export const selectableDecisionSectionEndPointListSelector = createDeepEqualSelector(
  [selectableDecisionSectionEndPointList],
  selectableDecisionSectionEndPointList => {
    return selectableDecisionSectionEndPointList;
  }
);

export const decisionSectionRouteCodeSelector = createDeepEqualSelector(
  [decisionSectionRouteCode],
  decisionSectionRouteCode => {
    return decisionSectionRouteCode;
  }
);
