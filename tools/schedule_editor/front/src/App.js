import React from 'react';
import {BrowserRouter} from 'react-router-dom';

import {createStore, combineReducers} from 'redux'
import {Provider} from 'react-redux'
import {routerReducer} from 'react-router-redux'

import Routes from './Routes';


const store = createStore(
  combineReducers({
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
