import React from 'react';
import PropTypes from 'prop-types';

import connect from 'react-redux/es/connect/connect';
import { bindActionCreators } from 'redux';

import Card from '@material-ui/core/Card';
import CardHeader from '@material-ui/core/CardHeader';
import CardContent from '@material-ui/core/CardContent';
import Button from '@material-ui/core/Button';
import Dialog from '@material-ui/core/Dialog';
import DialogTitle from '@material-ui/core/DialogTitle';
import IconButton from '@material-ui/core/IconButton';
import CloseIcon from '@material-ui/icons/Close';
import AddIcon from '@material-ui/icons/Add';

import * as ScheduleEditorActions from '../../../redux/Actions/ScheduleEditorActions';

import RouteList from './RouteList';
import RouteCodeEditor from './RouteCodeEditor/RouteCodeEditor';

class RouteListBox extends React.Component {
  constructor(props) {
    super(props);

    this.openAddRouteModal = this.openAddRouteModal.bind(this);
    this.closeAddRouteModal = this.closeAddRouteModal.bind(this);
  }

  openAddRouteModal() {
    this.props.scheduleEditorActions.setIsAddRouteModalOpen(true);
  }

  closeAddRouteModal() {
    this.props.scheduleEditorActions.setIsAddRouteModalOpen(false);
  }

  render() {

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
      height: 'calc(100% - 64px)'
    };

    const modalContent = {
      position: 'relative',
      height: window.innerHeight * 0.9
    };
    return (
      <div style={wrapper}>
        <Card style={CardStyle} id="RouteListCard">
          <CardHeader
            action={
              <Button
                variant="contained"
                color="primary"
                onClick={this.openAddRouteModal}
              >
                <AddIcon style={{ color: 'white', marginRight: '5px' }} />
                Add Route
              </Button>
            }
            title="Route List"
            id="RouteListCardHeader"
          />
          <CardContent style={contentStyle}>
            <RouteList />
          </CardContent>
        </Card>
        <Dialog
          open={this.props.isAddRouteModalOpen}
          onClose={this.closeAddRouteModal}
          fullWidth={true}
          maxWidth="xl"
        >
          <DialogTitle id="alert-dialog-slide-title">
            <IconButton
              color="inherit"
              onClick={this.closeAddRouteModal}
              aria-label="Close"
            >
              <CloseIcon />
            </IconButton>
          </DialogTitle>
          <div style={modalContent}>
            <RouteCodeEditor />
          </div>
        </Dialog>
      </div>
    );
  }
}

RouteListBox.propTypes = {
  isAddRouteModalOpen: PropTypes.bool,
  scheduleEditorActions: PropTypes.object
};
const mapStateSelectEndPoint = state => ({
  isAddRouteModalOpen: state.scheduleEditor.getIsAddRouteModalOpen()
});
const mapDispatchSelectEndPoint = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export default connect(
  mapStateSelectEndPoint,
  mapDispatchSelectEndPoint
)(RouteListBox);
