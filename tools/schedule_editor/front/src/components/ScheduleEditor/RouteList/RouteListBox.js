import React from 'react';
import connect from "react-redux/es/connect/connect";
import {bindActionCreators} from "redux";

import Card from '@material-ui/core/Card';
import CardHeader from '@material-ui/core/CardHeader';
import CardContent from '@material-ui/core/CardContent';
import Button from '@material-ui/core/Button';
import Dialog from '@material-ui/core/Dialog';
import DialogTitle from '@material-ui/core/DialogTitle';
import DialogActions from '@material-ui/core/DialogActions';

import * as ScheduleEditorActions from "../../../redux/Actions/ScheduleEditorActions";

import RouteList from './RouteList';
import RouteCodeEditor from '../RouteCodeEditor/RouteCodeEditor';


class RouteListBox extends React.Component {

  constructor(props) {
    super(props);
    this.state = {
      listHeight: 0
    };

    this.routeCodeEditorModalOpen = this.routeCodeEditorModalOpen.bind(this);
    this.closeModal = this.closeModal.bind(this);
    this.resize = this.resize.bind(this);
  }

  componentDidMount() {
    this.setState({listHeight: document.getElementById("RouteListCard").clientHeight - document.getElementById("RouteListCardHeader").clientHeight});
    document.getElementById("RouteListCard").addEventListener('resize', this.resize, false);
  }

  resize() {
    this.setState({listHeight: document.getElementById("RouteListCard").clientHeight - document.getElementById("RouteListCardHeader").clientHeight});
  }

  routeCodeEditorModalOpen() {
    this.props.scheduleEditorActions.setIsAddRouteModalOpen(true);
  }

  closeModal() {
    this.props.scheduleEditorActions.setIsAddRouteModalOpen(false);
  }

  render() {
    const cardContentHeight = this.state.listHeight;

    const wrapper = {
      padding: '5px',
      boxSizing: 'border-box',
      height: '100%'
    };

    const CardStyle = {
      height: '100%'
    };

    const contentStyle = {
      boxSizing: 'border-box',
      height: cardContentHeight
    };


    const modalContent = {
      position: 'relative',
      height: window.innerHeight * 0.9
    };
    return (
      <div style={wrapper}>
        <Card style={CardStyle}
              id="RouteListCard"
        >
          <CardHeader
            action={
              <Button variant="contained" color="primary" onClick={this.routeCodeEditorModalOpen}>
                Add Route
              </Button>
            }
            title="Route List"
            id="RouteListCardHeader"
          />
          <CardContent style={contentStyle}>
            <RouteList/>
          </CardContent>
        </Card>
        <Dialog
          open={this.props.isAddRouteModalOpen}
          onClose={this.closeModal}
          fullWidth={true}
          maxWidth='xl'
        >
          <DialogTitle id="alert-dialog-slide-title">
            {"Select Route"}
          </DialogTitle>
          <div style={modalContent}>
            <RouteCodeEditor/>
          </div>
          <DialogActions>
            <Button onClick={this.closeModal}>
              Cancel
            </Button>
          </DialogActions>
        </Dialog>
      </div>

    );
  }
}

const mapStateSelectEndPoint = (state) => ({
  isAddRouteModalOpen: state.scheduleEditor.getIsAddRouteModalOpen(),
});
const mapDispatchSelectEndPoint = (dispatch) => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch),
});
export default connect(mapStateSelectEndPoint, mapDispatchSelectEndPoint)(RouteListBox);


