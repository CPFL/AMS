export const REDUX = {
  ACTION_TYPE: {
    //Page
    //ScheduleEditor
    SET_MAP_DATA_SCHEDULE_EDITOR: 'setMapDataScheduleEditor',
    //Modal
    SET_IS_IMPORT_DATA_MODAL_OPEN: 'setIsImportDataModalModal',
    SET_IS_ADD_ROUTE_MODAL_OPEN: 'setIsAddRouteModalOpen',
    SET_IS_ADD_SCHEDULE_MODAL_OPEN: 'setIsAddScheduleModalOpen',

    //Route Code Editor
    SET_ACTIVE_STEP_ROUTE_CODE_EDITOR: 'setActiveStepRouteCodeEditor',
    BACK_STEP_ROUTE_CODE_EDITOR: 'backStepRouteCodeEditor',
    SET_IS_BACK_ROUTE_CODE_EDITOR: 'setIsBackScheduleEditor',
    SET_START_POINT_ROUTE_CODE_EDITOR: 'setStartPointRouteCodeEditor',
    SET_LANE_LIST_ROUTE_CODE_EDITOR: 'setLaneListRouteCodeEditor',
    SET_END_POINT_ROUTE_CODE_EDITOR: 'setEndPointRouteCodeEditor',
    SET_START_POINT_AND_LANE_LIST_SCHEDULE_EDITOR:
      'setStartPointAndLaneListScheduleEditor',
    CLEAR_ROUTE_CODE_DATA: 'clearRouteCodeData',
    SAVE_ROUTE_CODE: 'saveRouteCode',
    SAVE_AND_ANOTHER_SELECT_ROUTE_CODE: 'saveAndAnotherSelectRouteCode',
    RESET_ROUTE_CODE: 'resetRouteCode',
    //Schedule Editor
    SET_ACTIVE_STEP_SCHEDULE_EDITOR: 'setActiveStepScheduleEditor',
    SER_CURRENT_ROUTE_CODE_SCHEDULE: 'setCurrentRouteCodeSchedule',
    SET_CHECKED_SEND_ENGAGE: 'setCheckedSendEngage',
    SET_WAIT_TIME: 'setWaitTime',
    SET_CURRENT_EDIT_CHANGE_ROUTE_LIST: 'setCurrentEditChangeRouteList',
    SAVE_SCHEDULE: 'saveSchedule',
    // Change Route Editor
    SET_CHANGE_ROUTE_ACTIVE_STEP_NEXT: 'setChangeRouteActiveStepNext',
    SET_CHANGE_ROUTE_ACTIVE_STEP_PREVIOUS: 'setChangeRouteActiveStepNext',
    SET_CHANGE_ROUTE_ACTIVE_STEP_RESET: 'setChangeRouteActiveStepNext',

    SET_CHANGE_ROUTE_IS_BACK: 'setChangeRouteIsBack',
    SET_CHANGE_ROUTE_START_POINT: 'setChangeRouteStartPoint',
    SET_CHANGE_ROUTE_LANE_LIST: 'setChangeRouteLaneList',
    SET_CHANGE_ROUTE_END_POINT: 'setChangeRouteEndPoint'
  }
};
