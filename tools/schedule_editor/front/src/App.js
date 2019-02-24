import React from 'react';
import { BrowserRouter } from 'react-router-dom';

import { createStore, combineReducers } from 'redux';
import { Provider } from 'react-redux';
import { routerReducer } from 'react-router-redux';

import Routes from './Routes';

import { scheduleEditorReducer } from './redux/Reducers/ScheduleEditorReducer';

const store = createStore(
  combineReducers({
    scheduleEditor: scheduleEditorReducer,
    routing: routerReducer
  })
);

const app = () => {
  return (
    <Provider store={store}>
      <BrowserRouter basename={WP_BASE_HREF}>
        <Routes />
      </BrowserRouter>
    </Provider>
  );
};
export default app;
