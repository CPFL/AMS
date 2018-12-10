import React from 'react';
import {Route, withRouter} from 'react-router-dom';

import ScheduleEditor from './components/ScheduleEditor/ScheduleEditor'
import Header from './components/Header/Header'

import {AppBar} from '@material-ui/core';

const Routes = () => {

  let wrapper = {
    position: 'absolute',
    top: '64px',
    right: 0,
    bottom: 0,
    left: 0
  };

  return (
    <div>
      <Header/>
      <div style={wrapper}>
        <Route exact path="/" component={ScheduleEditor}/>
      </div>
    </div>
  )

};


export default withRouter(Routes);
