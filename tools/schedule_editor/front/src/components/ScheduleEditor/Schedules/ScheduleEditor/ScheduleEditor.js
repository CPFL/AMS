import React from 'react';
import Grid from '@material-ui/core/Grid';

import ScheduleEditProcess from './ScheduleEditProcess';
import Map3DManager from './Maps/Map3D/Map3DManager';

export default class ScheduleEditor extends React.Component {
  constructor(props) {
    super(props);
  }

  render() {
    const wrapper = {
      position: 'absolute',
      top: 0,
      right: 0,
      bottom: 0,
      left: 0
    };

    const Map3DWrapper = {
      padding: '5px',
      boxSizing: 'border-box',
      height: '100%'
    };

    return (
      <div style={wrapper}>
        <Grid container style={{ height: '100%' }}>
          <Grid item xs={7} style={{ position: 'relative' }}>
            <ScheduleEditProcess />
          </Grid>
          <Grid item xs={5} style={{ position: 'relative' }}>
            <div style={Map3DWrapper}>
              <div style={{ height: '99%', width: '99%' }}>
                <Map3DManager />
              </div>
            </div>
          </Grid>
        </Grid>
      </div>
    );
  }
}
