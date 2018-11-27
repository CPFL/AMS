import React from 'react';
import {Route, withRouter} from 'react-router-dom';

import RouteCodeEditor from './components/Page/RouteCodeEditor'

import {AppBar, Toolbar, Typography} from '@material-ui/core';

const Routes = () => {

  const styles = {
    root: {
      flexGrow: 1,
    },
    grow: {
      flexGrow: 1,
    },
    menuButton: {
      marginLeft: -12,
      marginRight: 20,
    },
  };

  let wrapper = {
    position: 'absolute',
    //marginTop: '64px',
    top: '64px',
    right: 0,
    bottom: 0,
    left: 0
  };

  return (
    <div>
      <div style={styles.root}>
        <AppBar position="static">
          <Toolbar>
            <Typography variant="h6" color="inherit" style={styles.grow}>
              Route Code Editor
            </Typography>
          </Toolbar>
        </AppBar>
      </div>
      <div style={wrapper}>
        <Route exact path="/" component={RouteCodeEditor}/>
      </div>
    </div>
  )

};


export default withRouter(Routes);
