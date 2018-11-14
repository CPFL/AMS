export const VEHICLE_MANAGER = {
  JSG2011: {
    LATLNG: {
      EPSG: 'EPSG:6668',
      OPTION: '+proj=longlat +ellps=GRS80 +no_defs',
    },
    RECTANGULAR_PLANE: {
      NAGASAKI: {
        NAME: "長崎・鹿児島一部",
        EPSG: 'EPSG:6669',
        OPTION: '+proj=tmerc +lat_0=33 +lon_0=129.5 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      KYUSYU: {
        NAME: "九州地方",
        EPSG: 'EPSG:6670',
        OPTION: '+proj=tmerc +lat_0=33 +lon_0=131 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      WEST_CHUGOKU: {
        NAME: "中国地方西部",
        EPSG: 'EPSG:6671',
        OPTION: '+proj=tmerc +lat_0=36 +lon_0=132.1666666666667 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      SHIKOKU: {
        NAME: "四国地方",
        EPSG: 'EPSG:6672',
        OPTION: '+proj=tmerc +lat_0=33 +lon_0=133.5 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      EAST_CHUGOKU: {
        NAME: "中国地方東部",
        EPSG: 'EPSG:6673',
        OPTION: '+proj=tmerc +lat_0=36 +lon_0=134.3333333333333 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      KINKI: {
        NAME: "近畿地方",
        EPSG: 'EPSG:6674',
        OPTION: '+proj=tmerc +lat_0=36 +lon_0=136 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      WEST_CHUBU: {
        NAME: "中部地方西部",
        EPSG: 'EPSG:6675',
        OPTION: '+proj=tmerc +lat_0=36 +lon_0=137.1666666666667 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      EAST_CHUBU: {
        NAME: "中部地方東部",
        EPSG: 'EPSG:6676',
        OPTION: '+proj=tmerc +lat_0=36 +lon_0=138.5 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      KANTO: {
        NAME: "関東地方",
        EPSG: 'EPSG:6677',
        OPTION: '+proj=tmerc +lat_0=36 +lon_0=139.8333333333333 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      TOHOKU: {
        NAME: "東北地方",
        EPSG: 'EPSG:6678',
        OPTION: '+proj=tmerc +lat_0=40 +lon_0=140.8333333333333 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      WEST_HOKKAIDO: {
        NAME: "北海道西部",
        EPSG: 'EPSG:6679',
        OPTION: '+proj=tmerc +lat_0=44 +lon_0=140.25 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      MIDDLE_HOKKAIDO: {
        NAME: "北海道中央",
        EPSG: 'EPSG:6680',
        OPTION: '+proj=tmerc +lat_0=44 +lon_0=142.25 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      EAST_HOKKAIDO: {
        NAME: "北海道東部",
        EPSG: 'EPSG:6681',
        OPTION: '+proj=tmerc +lat_0=44 +lon_0=144.25 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      OGASAWARA: {
        NAME: "小笠原諸島",
        EPSG: 'EPSG:6682',
        OPTION: '+proj=tmerc +lat_0=26 +lon_0=142 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      OKINAWA: {
        NAME: "沖縄",
        EPSG: 'EPSG:6683',
        OPTION: '+proj=tmerc +lat_0=26 +lon_0=127.5 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      ISHIGAKI: {
        NAME: "石垣島周辺",
        EPSG: 'EPSG:6684',
        OPTION: '+proj=tmerc +lat_0=26 +lon_0=124 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      DAITO: {
        NAME: "大東島周辺",
        EPSG: 'EPSG:6685',
        OPTION: '+proj=tmerc +lat_0=26 +lon_0=131 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      OKINOTORI: {
        NAME: "沖ノ鳥島周辺",
        EPSG: 'EPSG:6686',
        OPTION: '+proj=tmerc +lat_0=20 +lon_0=136 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      },
      MINAMITORI: {
        NAME: "南鳥島周辺",
        EPSG: 'EPSG:6687',
        OPTION: '+proj=tmerc +lat_0=26 +lon_0=154 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs',
      }
    }
  },
  LIST_DISPLAY: {
    ENABLE: 'Enable',
    DISABLE: 'Disable'
  },
  SCHEDULE: {
    STATUS: {
      Running: [
        "send_lane_waypoint_array"
      ],
      Stopping: [
        "activate",
        "start_mission",
        "check_engage_button_status",
        "send_engage"
      ]
    }
  }
};

export const ADASMAP_LOADER = {

  URL_PREFIX: "/static/route_code_editor/data/maps/vectors/"

};

export const MAP_MANAGER = {
  VEHICLE_MARKER_INFO: {
    active: {
      color: "#ff090f"
    },
    inactive: {
      color: "#ff090f"
    },
    waiting_for_decision_maker_state_drive_ready: {
      color: "#ff090f",
      style: "mapbox-marker"
    },
    waiting_for_decision_maker_state_wait_order: {
      color: "#06a2ff"
    },
    mission_started: {
      color: "#ff090f"
    },
    waiting_for_engaged: {
      color: "#13e00b"
    },
    disconnect: {
      color: "#000000"
    }
  },
  WAYPOINT_INFO: {
    activate: {
      color: "#13e00b"
    },
    move: {
      color: "#0dd0ff"
    },
    stop: {
      color: "#ff2117"
    },
    inactive: {
      color: "#0dd0ff"
    }
  },
  ICON: {
    default: "M29.395,0H17.636c-3.117,0-5.643,3.467-5.643,6.584v34.804c0,3.116,2.526,5.644,5.643,5.644h11.759 c3.116,0,5.644-2.527,5.644-5.644V6.584C35.037,3.467,32.511,0,29.395,0z M34.05,14.188v11.665l-2.729,0.351v-4.806L34.05,14.188z M32.618,10.773c-1.016,3.9-2.219,8.51-2.219,8.51H16.631l-2.222-8.51C14.41,10.773,23.293,7.755,32.618,10.773z M15.741,21.713 v4.492l-2.73-0.349V14.502L15.741,21.713z M13.011,37.938V27.579l2.73,0.343v8.196L13.011,37.938z M14.568,40.882l2.218-3.336 h13.771l2.219,3.336H14.568z M31.321,35.805v-7.872l2.729-0.355v10.048L31.321,35.805z",
    milee_shimz: "M29.395,0H17.636c-3.117,0-5.643,3.467-5.643,6.584v34.804c0,3.116,2.526,5.644,5.643,5.644h11.759 c3.116,0,5.644-2.527,5.644-5.644V6.584C35.037,3.467,32.511,0,29.395,0z M34.05,14.188v11.665l-2.729,0.351v-4.806L34.05,14.188z M32.618,10.773c-1.016,3.9-2.219,8.51-2.219,8.51H16.631l-2.222-8.51C14.41,10.773,23.293,7.755,32.618,10.773z M15.741,21.713 v4.492l-2.73-0.349V14.502L15.741,21.713z M13.011,37.938V27.579l2.73,0.343v8.196L13.011,37.938z M14.568,40.882l2.218-3.336 h13.771l2.219,3.336H14.568z M31.321,35.805v-7.872l2.729-0.355v10.048L31.321,35.805z",
    logiee_shimz: "m4500 6e3c0 283 217 500 500 500s500-217 500-500m-1e3 -2500c0-283 217-500 500-500s500 217 500 500zm-1750 2500h4500m-4500-2500h4500m-4501-1h4503v3h-4503zm250 4951c-248-496-496-1483-500-3450v-25-25c4-1967 252-2954 500-3450 207-414 756-656 1507-728 35-244 238-422 494-422 257 0 459 178 494 422 750 72 1299 314 1506 728 248 496 496 1483 500 3450v25 25c-4 1967-252 2954-500 3450-250 500-1e3 750-2000.5 750s-1750.5-250-2000.5-750zm3801-206c224-468 448-1403 450-3269-2-1866-226-2801-450-3269-225-471-900-706-1800-706v-150h-1v150c-900 0-1575 235-1800 706-224 468-448 1403-450 3269 2 1866 226 2801 450 3269 225 471 900 706 1800.5 706s1575.5-235 1800.5-706z",
  },
  OBJECT: {
    LAMP_LOCATION: {
      LAT: 35.662385,
      LNG: 139.796821
    },
    LAT_PER_1M: 0.0000090133729745762,
    LNG_PER_1M: 0.000010966404715491394
  }
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

export const MAP_3D = {
  SCENE_TYPE: {
    VEHICLE_COLLADA: "vehicleCollada",
  },
  MODEL_PATH: {
    VEHICLE: {
      logiee_shimz: "/static/route_code_editor/data/model/vehicle/logiee.dae",
      milee_shimz: "/static/route_code_editor/data/model/vehicle/milee.dae",
      postee_shimz: "/static/route_code_editor/data/model/vehicle/milee.dae",
      default: "/static/route_code_editor/data/model/vehicle/default.dae"
    }
  },

};


export const REDUX = {
  ACTION_TYPE: {
    //Parameter Set
    SET_MAP_PARAMETER: "setMapParameter",
    SET_CENTER_VEHICLE_ID: "setCenterVehicleId",
    SET_FOLLOWING_CENTER_VEHICLE: "setFollowingCenterVehicle",
    SET_PUBLISH_LAT_LNG: "setPublishLatLng",

    //Vehicle Status
    SET_VEHICLE_LIST: "setVehicleList",
    UPDATE_VEHICLE_LIST: "updateVehicleList",
    UPDATE_VEHICLE_STATUS_LIST: "updateVehicleStatusList",
    UPDATE_VEHICLE_STATUS_NOT_UPDATE: "updateVehicleStatusNotUpdate",

    //Schedule
    UPDATE_VEHICLE_SCHEDULES: "updateVehicleSchedules",
    UPDATE_VEHICLE_ROUTES: "updateVehicleRoutes",
    SET_VEHICLE_ROUTE: "setVehicleRoute",
    DELETE_VEHICLE_ROUTE: "deleteVehicleRoute",

    //Coordinate
    SET_COORDINATE_SYSTEM: "setCoordinateSystem",

    //Notification
    INIT_NOTIFICATION_LIST: "initNotificationList",
    UPDATE_NOTIFICATION_LIST: "updateNotificationList",

    //Iot Publish
    PUBLISH_MESSAGE: "publishMessage",
    IOT_PUBLISH_GET_SCHEDULES: "iotPublishGetSchedules",
    IOT_PUBLISH_GET_ROUTE: "iotPublishGetRoute",
    IOT_PUBLISH_ACTIVATION: "iotPublishActivation",
    IOT_PUBLISH_INACTIVATION: "iotPublishInactivation",
    IOT_PUBLISH_CHANGE_PATOLAMP_STATUS: "iotPublishChangePatolampStatus",
    IOT_PUBLISH_CHANGE_PATOLAMP_STATUS_MOCK: "iotPublishChangePatolampStatusMock",
    IOT_PUBLISH_LOCATION_AS_LAT_LNG: "iotPublishLocationAsLatLng",
    IOT_PUBLISH_ENGAGE: "iotPublishEngage",

    //Object3D Status
    SET_PATOLAMP_STATUS: "setPatoLampStatus",
    SET_PATOLAMP_RADIUS: "setPatoLampRadius",
    SET_SHUTTER_STATUS: "setShutterStatus",
    SET_SHUTTER_OPERATION: "setShutterOperation",

    //3D Object3D And Map
    SIZE_UP_PCD: "sizeUpPCD",
    SIZE_DOWN_PCD: "sizeDownPCD",

    //Page
    //RouteCodeEditor
    SET_HEIGHT_AND_WIDTH_ROUTE_CODE_EDITOR: "setHeightAndWidthRouteCodeEditor",
    SET_ACTIVE_STEP_ROUTE_CODE_EDITOR: "setActiveStepRouteCodeEditor",
    SET_IS_BACK_ROUTE_CODE_EDITOR: "setIsBackRouteCodeEditor",

    //Test
    SET_MOCK_MODE: "setMockMode",
    SET_LOCATION_ONLY_MODE: "setLocationOnlyMode",
    SET_PATOLAMP_STATUS_FROM_MOCK: "setPatoLampStatusFromMock",
    SET_SHUTTER_STATUS_FROM_MOCK: "setShutterStatusFromMock",
    SET_WAYPOINTS_FROM_LOCAL_FILE: "setWaypointsFromLocalFile",
    SET_PCD_FROM_LOCAL_FILE: "setPCDFromLocalFile",
    SET_ADASMAP_FROM_FILE: "setAdasmapFromFile",
    UPDATE_VEHICLE_STATUS_LIST_TEST: "updateVehicleStatusListTest"

  }
};