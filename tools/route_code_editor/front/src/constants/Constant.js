
export const ADASMAP_LOADER = {

  URL_PREFIX: "/static/route_code_editor/data/maps/vectors/"

};

export const OBJECT = {
  SHUTTER: {
    STATUS: {
      OPENING: "opening",
      OPENED: "opened",
      CLOSING: "closing",
      CLOSED: "closed",
      UNDEFINED: "undefined"
    },
    IMG_PATH: {
      opening: "/static/route_code_editor/img/maps/OPENING.png",
      opened: "/static/route_code_editor/img/maps/OPENED.png",
      closing: "/static/route_code_editor/img/maps/CLOSING.png",
      closed: "/static/route_code_editor/img/maps/CLOSED.png"
    },
    INITIAL_DATA: {
      ass1: {
        id: "ass1",
        type: "shutter",
        modelPath: "/static/route_code_editor/data/model/object/shutter.3ds",
        position: {
          "x": -3315.2,
          "y": -37494.0,
          "z": 1.7504
        },
        rotationZ: 5.43,
        scale: {
          "x": 0.00103,
          "y": 0.001,
          "z": 0.0009
        },
        location: {
          lat: 35.662044,
          lng: 139.796716
        },
        initState: [],
        status: "closed",
        openPercentage: 0
      },
      ass2: {
        id: "ass2",
        type: "shutter",
        modelPath: "/static/route_code_editor/data/model/object/shutter.3ds",
        position: {
          "x": -3278,
          "y": -37404,
          "z": 0.001
        },
        rotationZ: 5.43 + (Math.PI / 2),
        scale: {
          "x": 0.00103,
          "y": 0.001,
          "z": 0.0009
        },
        location: {
          lat: 35.662894,
          lng: 139.797148
        },
        initState: [
          {
            "type": "shutterOpenPercentage",
            "percentage": 1.0
          }
        ],
        status: "opened",
        openPercentage: 1.0
      }
    }
  },
  PATOLAMP: {
    AREA_LINE: {
      COLOR: {
        ON: "#13e00b",
        OFF: "#ff090f"
      }
    },
    IMG_PATH: {
      PATOLAMP_ON: "/static/route_code_editor/img/maps/pato_lamp_on.png",
      PATOLAMP_OFF: "/static/route_code_editor/img/maps/pato_lamp_off.png"
    },
    INITIAL_DATA: {
      lt1: {
        id: "lt1",
        type: "patolamp",
        modelPath: "/static/route_code_editor/data/model/object/patolamp.3ds",
        position: {
          "x": -3216,
          "y": -37392,
          "z": 1.5
        },
        rotationZ: 0,
        scale: {
          "x": 0.00903,
          "y": 0.009,
          "z": 0.0081
        },
        location: {
          lat: 35.663146,
          lng: 139.797794
        },
        initState: [],
        radius: 15.0,
        status: "off"
      },
      lt2: {
        id: "lt2",
        type: "patolamp",
        modelPath: "/static/route_code_editor/data/model/object/patolamp.3ds",
        position: {
          "x": -3305.7307,
          "y": -37456.0078,
          "z": 1.5
        },
        rotationZ: 0,
        scale: {
          "x": 0.00903,
          "y": 0.009,
          "z": 0.0081
        },
        location: {
          lat: 35.662385,
          lng: 139.796821
        },
        initState: [],
        radius: 15.0,
        status: "off"
      },
      lt3: {
        id: "lt3",
        type: "patolamp",
        modelPath: "/static/route_code_editor/data/model/object/patolamp.3ds",
        position: {
          "x": -3326,
          "y": -37480,
          "z": 2.7
        },
        rotationZ: 0,
        scale: {
          "x": 0.00903,
          "y": 0.009,
          "z": 0.0081
        },
        location: {
          lat: 35.662126,
          lng: 139.796517
        },
        initState: [],
        radius: 15.0,
        status: "off"
      },
    }
  },
  INVISIBLE_LINE: 17,
};


export const REDUX = {
  ACTION_TYPE: {

    //Page
    //RouteCodeEditor
    SET_HEIGHT_AND_WIDTH_ROUTE_CODE_EDITOR: "setHeightAndWidthRouteCodeEditor",
    SET_ACTIVE_STEP_ROUTE_CODE_EDITOR: "setActiveStepRouteCodeEditor",
    SET_IS_BACK_ROUTE_CODE_EDITOR: "setIsBackRouteCodeEditor",
    SET_WAYPOINT_ROUTE_CODE_EDITOR: "setWaypointRouteCodeEditor",
    SET_LANE_ROUTE_CODE_EDITOR: "setLaneRouteCodeEditor",
    REFLECT_MAP_DATA_ROUTE_CODE_EDITOR: "reflectMapDataRouteCodeEditor",
    SET_START_POINT_ROUTE_CODE_EDITOR: "setStartPointRouteCodeEditor",
    SET_LANE_LIST_ROUTE_CODE_EDITOR: "setLaneListRouteCodeEditor",
    SET_END_POINT_ROUTE_CODE_EDITOR: "setEndPointRouteCodeEditor",
    CLEAR_ROUTE_CODE_DATA_ROUTE_CODE_EDITOR: "clearRouteCodeDataRouteCodeEditor",


    //Test
    SET_WAYPOINTS_FROM_LOCAL_FILE: "setWaypointsFromLocalFile",
    SET_PCD_FROM_LOCAL_FILE: "setPCDFromLocalFile",
    SET_ADASMAP_FROM_FILE: "setAdasmapFromFile",

  }
};