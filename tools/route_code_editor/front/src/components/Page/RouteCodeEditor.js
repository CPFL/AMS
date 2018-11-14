import React from 'react';
import {
  Card,
  CardTitle,
  CardActions,
  Button,
  Grid,
  Cell
} from 'react-mdl';
import {connect} from "react-redux";

import Map3DManager from "../Maps/Map3D/Map3DManager";
import * as RouteCodeEditorActions from "../../redux/Actions/RouteCodeEditorActions";
import {bindActionCreators} from "redux";
import RouteCodeMakeTabs from "../Tabs/RouteCodeMakeTab";

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

    let label_input = {
      display: "none"
    };

    return (
      <div style={wrapper}>
        <Grid style={{height: '95%'}}>
          <Cell col={6} style={cellStyle}>
            <Card shadow={0} style={mapWrapper} id="card">
              <CardTitle id="cardTitle">
                <label htmlFor="pcd_reader" className="file_reader">
                  Select PCD
                  <input type="file" style={label_input} id="pcd_reader"/>
                </label>
                <label htmlFor="waypoints_reader" className="file_reader">
                  Select Waypoints
                  <input type="file" style={label_input} id="waypoints_reader"/>
                </label>
                <label htmlFor="lane_reader" className="file_reader">
                  Select Lane
                  <input type="file" style={label_input} id="lane_reader"/>
                </label>
                <br />
                <Button style={{marginLeft: "20px"}}>Import Select Data</Button>
                <Button>Clear</Button>
              </CardTitle>
              <CardActions border id="cardAction">
                <Map3DManager/>
              </CardActions>
            </Card>
          </Cell>
          <Cell col={6}>
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