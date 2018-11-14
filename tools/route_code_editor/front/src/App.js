import React from 'react';
import {BrowserRouter, Router} from 'react-router-dom';
import {createHistory} from 'history'

import {createStore, combineReducers, applyMiddleware} from 'redux'
import {Provider} from 'react-redux'
import {routerReducer, routerMiddleware} from 'react-router-redux'

import {settingsReducer, operatingMapFromLocalReducer} from './redux/Reducers/SettingsReducer';
import {routeCodeEditorReducer} from './redux/Reducers/RouteCodeEditorReducer';


import Routes from './Routes';


const store = createStore(
  combineReducers({
    settings: settingsReducer,
    operatingMapFromLocal: operatingMapFromLocalReducer,
    routeCodeEditor: routeCodeEditorReducer,
    routing: routerReducer
  })
);


export default () => (
  <Provider store={store}>
    <BrowserRouter basename={WP_BASE_HREF}>
      <Routes/>
    </BrowserRouter>
  </Provider>
);
