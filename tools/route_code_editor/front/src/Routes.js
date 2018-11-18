import React, {Component} from 'react';
import {Route, Link, withRouter} from 'react-router-dom';

import RouteCodeEditor from './components/Page/RouteCodeEditor'

import {AppBar, Toolbar, Typography, Button, IconButton} from '@material-ui/core';
import MenuIcon from '@material-ui/icons/Menu';

/*
const listofPages = [
'/login',
'/register',
'/recover',
'/lock',
'/notfound',
'/error500',
'/maintenance'
];
*/

/*
const RouteHideDrawer = ({component: Component, ...rest}) => (
  <Route {...rest} render={() => {
    if (document.querySelector('.mdl-layout__drawer')) {
      document.querySelector('.mdl-layout__obfuscator').classList.remove('is-visible');
      document.querySelector('.mdl-layout__drawer').classList.remove('is-visible');
    }
    return <Component/>
  }}/>
);
*/

const Routes = ({location}) => {

  //const currentKey = location.pathname.split('/')[1] || '/';
  //const timeout = { enter: 500, exit: 500 };

  // Animations supported
  //      'rag-fadeIn'
  //      'rag-fadeInUp'
  //      'rag-fadeInDown'
  //      'rag-fadeInRight'
  //      'rag-fadeInLeft'
  //      'rag-fadeInUpBig'
  //      'rag-fadeInDownBig'
  //      'rag-fadeInRightBig'
  //      'rag-fadeInLeftBig'
  //      'rag-zoomBackDown'
  //const animationName = 'rag-fadeIn';

  /*
  if(listofPages.indexOf(location.pathname) > -1) {
  return (
  // Page Layout component wrapper
  <BasePage>
  <Switch location={location}>
  <Route path="/login" component={Login}/>
  <Route path="/register" component={Register}/>
  <Route path="/recover" component={Recover}/>
  <Route path="/lock" component={Lock}/>
  <Route path="/notfound" component={NotFound}/>
  <Route path="/error500" component={Error500}/>
  <Route path="/maintenance" component={Maintenance}/>
  </Switch>
  </BasePage>
)
}

else {
*/

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
            <IconButton style={styles.menuButton} color="inherit" aria-label="Menu">
              <MenuIcon/>
            </IconButton>
            <Typography variant="h6" color="inherit" style={styles.grow}>
              Route Code Editor
            </Typography>
            {/*<Button color="inherit">Login</Button>*/}
          </Toolbar>
        </AppBar>
      </div>
      <div style={wrapper}>
        <Route exact path="/" component={RouteCodeEditor}/>
      </div>
    </div>
  )


  /*
  return (
    <Layout fixedHeader>
      <Header title="Route Code Editor" style={{backgroundColor: "#002041"}}/>
      <Drawer title="Route Code Editor">
        <Navigation>
          <Link to="/">Home</Link>
          <Link to="/route-code-editor">Route Code Editor</Link>
          <Link to="/settings">Settings</Link>
        </Navigation>
      </Drawer>
      <Content>
        <RouteHideDrawer exact path="/" component={RouteCodeEditor}/>
        <RouteHideDrawer path="/route-code-editor" component={RouteCodeEditor}/>
        <RouteHideDrawer path="/settings" component={Settings}/>
      </Content>
    </Layout>
  )
  */
};


export default withRouter(Routes);
