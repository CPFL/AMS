import React from 'react';
import Grid from '@material-ui/core/Grid';

import Map3DManager from './Maps/Map3D/Map3DManager';
import CreateRouteCodeProcess from './Process/CreateRouteCodeProcess';

export default class RouteCodeEditor extends React.Component {
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
          <Grid item xs style={{ position: 'relative' }}>
            <CreateRouteCodeProcess />
          </Grid>
          <Grid item xs style={{ position: 'relative' }}>
            <div style={Map3DWrapper}>
              <div style={{ height: '100%', width: '100%' }}>
                <Map3DManager />
              </div>
            </div>
          </Grid>
        </Grid>
      </div>
    );
  }
}
