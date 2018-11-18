import React from 'react';
import {
  Card,
  CardTitle,
  CardActions,
  Grid,
  Cell
} from 'react-mdl';
import {connect} from "react-redux";
import {bindActionCreators} from "redux";

import Map3DManager from "../RouteCodeEditor/Maps/Map3D/Map3DManager";
import RouteCodeMakeTabs from "../RouteCodeEditor/Tabs/RouteCodeMakeTab";
import ImportFile from '../RouteCodeEditor/ImportFile/ImportFile';
import * as RouteCodeEditorActions from "../../redux/Actions/RouteCodeEditorActions";


class RouteCodeEditor extends React.Component {

  constructor(props) {
    super(props);
  }

  componentDidMount() {
    this.props.routeCodeEditorActions.setHeightAndWidth(
      (document.getElementById("card").clientHeight - document.getElementById("cardTitle").clientHeight) * 0.95,
      document.getElementById("cardAction").clientWidth
    );
    window.onresize = this.resize.bind(this);
  }

  resize() {
    this.props.routeCodeEditorActions.setHeightAndWidth(
      (document.getElementById("card").clientHeight - document.getElementById("cardTitle").clientHeight) * 0.95,
      document.getElementById("cardAction").clientWidth
    );
  }

  render() {

    let wrapper = {
      position: 'absolute',
      top: 0,
      right: 0,
      bottom: 0,
      left: 0
    };

    let cellStyle = {
      height: '100%',
      position: 'relative'
    };

    let mapWrapper = {
      width: '100%',
      height: '100%',
    };

    return (
      <div style={wrapper}>
        <Grid style={{height: '95%'}}>
          <Cell col={7} style={cellStyle}>
            <Card shadow={0} style={mapWrapper} id="card">
              <CardTitle id="cardTitle">
                <ImportFile/>
              </CardTitle>
              <CardActions border id="cardAction">
                <Map3DManager/>
              </CardActions>
            </Card>
          </Cell>
          <Cell col={5}>
            <RouteCodeMakeTabs />
          </Cell>
        </Grid>
      </div>

    );
  }
}

const mapState = () => ({});

const mapDispatch = (dispatch) => ({
  routeCodeEditorActions: bindActionCreators(RouteCodeEditorActions, dispatch),

});

export default connect(mapState, mapDispatch)(RouteCodeEditor);