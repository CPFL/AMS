import React from 'react';
import {connect} from "react-redux";
import {bindActionCreators} from "redux";

import Grid from '@material-ui/core/Grid';

import Map3DManager from './Maps/Map3D/Map3DManager'
import RouteList from './RouteList/RouteListBox';
import ScheduleBox from './Schedules/ScheduleBox';

import * as ScheduleEditorActions from "../../redux/Actions/ScheduleEditorActions";

export default class ScheduleEditor extends React.Component {

  constructor(props) {
    super(props);
  }

  render() {

    let wrapper = {
      position: 'absolute',
      top: 0,
      right: 0,
      bottom: 0,
      left: 0
    };

    const Map3DWrapper = {
      padding: '5px',
      boxSizing: 'border-box',
      height: '100%',
    };

    return (
      <div style={wrapper}>
        <Grid container style={{height: '100%'}}>
          <Grid item xs={7} style={{position: 'relative'}}>
            <Grid container style={{height: '50%'}}>
              <Grid item xs style={{position: 'relative'}}>
                <RouteList/>
              </Grid>
            </Grid>
            <Grid container style={{height: '50%'}}>
              <Grid item xs style={{position: 'relative'}}>
                <ScheduleBox/>
              </Grid>
            </Grid>
          </Grid>
          <Grid item xs={5} style={{position: 'relative'}}>
            <div style={Map3DWrapper}>
              <div style={{height: '100%', width: '100%'}} id="MapWrapper">
                <Map3DManager/>
              </div>
            </div>
          </Grid>
        </Grid>
      </div>
    );
  }
}

