import React from 'react';

import Card from '@material-ui/core/Card';
import CardHeader from '@material-ui/core/CardHeader';
import CardContent from '@material-ui/core/CardContent';
import Button from '@material-ui/core/Button';
import Dialog from '@material-ui/core/Dialog';
import ScheduleList from './ScheduleList';
import AddIcon from '@material-ui/icons/Add';

import ScheduleEditProcess from './ScheduleEditor/ScheduleEditor';
import DialogTitle from '@material-ui/core/DialogTitle/DialogTitle';
import IconButton from '@material-ui/core/IconButton/IconButton';
import CloseIcon from '@material-ui/icons/Close';
import PropTypes from 'prop-types';
import { bindActionCreators } from 'redux';
import * as ScheduleEditorActions from '../../../redux/Actions/ScheduleEditorActions';
import connect from 'react-redux/es/connect/connect';

class ScheduleBox extends React.Component {
  constructor(props) {
    super(props);

    this.openAddScheduleModal = this.openAddScheduleModal.bind(this);
    this.closeAddScheduleModal = this.closeAddScheduleModal.bind(this);
  }

  openAddScheduleModal() {
    this.props.scheduleEditorActions.setIsAddScheduleModalOpen(true);
  }

  closeAddScheduleModal() {
    this.props.scheduleEditorActions.setIsAddScheduleModalOpen(false);
  }

  render() {

    const wrapper = {
      padding: '5px',
      boxSizing: 'border-box',
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
        <Card style={{ height: '100%' }} id="ScheduleCard">
          <CardHeader
            action={
              <Button
                variant="contained"
                color="primary"
                onClick={this.openAddScheduleModal}
              >
                <AddIcon style={{ color: 'white', marginRight: '5px' }} />
                Add Schedule
              </Button>
            }
            title="Schedule"
            id="ScheduleCardHeader"
          />
          <CardContent style={contentStyle}>
            <ScheduleList />
          </CardContent>
        </Card>
        <Dialog
          open={this.props.isAddScheduleModalOpen}
          onClose={this.closeAddScheduleModal}
          fullWidth={true}
          maxWidth="xl"
        >
          <DialogTitle id="alert-dialog-slide-title">
            <IconButton
              color="inherit"
              onClick={this.closeAddScheduleModal}
              aria-label="Close"
            >
              <CloseIcon />
            </IconButton>
          </DialogTitle>
          <div style={modalContent}>
            <ScheduleEditProcess />
          </div>
        </Dialog>
      </div>
    );
  }
}

ScheduleBox.propTypes = {
  isAddScheduleModalOpen: PropTypes.bool,
  scheduleEditorActions: PropTypes.object
};
const mapStateSelectEndPoint = state => ({
  isAddScheduleModalOpen: state.scheduleEditor.getIsAddScheduleModalOpen()
});
const mapDispatchSelectEndPoint = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export default connect(
  mapStateSelectEndPoint,
  mapDispatchSelectEndPoint
)(ScheduleBox);
