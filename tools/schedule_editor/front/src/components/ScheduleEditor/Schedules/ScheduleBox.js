import React from 'react';

import Card from '@material-ui/core/Card';
import CardHeader from '@material-ui/core/CardHeader';
import CardContent from '@material-ui/core/CardContent';
import Button from '@material-ui/core/Button';
import Dialog from '@material-ui/core/Dialog';
import ScheduleList from './ScheduleList';
import AddIcon from '@material-ui/icons/Add';
import EditIcon from '@material-ui/icons/Edit';
import DeleteIcon from '@material-ui/icons/Delete';

import ScheduleEditProcess from './ScheduleEditor/ScheduleEditor';
import DialogTitle from '@material-ui/core/DialogTitle/DialogTitle';
import IconButton from '@material-ui/core/IconButton/IconButton';
import CloseIcon from '@material-ui/icons/Close';
import PropTypes from 'prop-types';
import { bindActionCreators } from 'redux';
import * as ScheduleEditorActions from '../../../redux/Actions/ScheduleEditorActions';
import connect from 'react-redux/es/connect/connect';
import CardActions from '@material-ui/core/CardActions/CardActions';

import ScheduleDownloadButton from './ScheduleDownloadButton';

class ScheduleBox extends React.Component {
  constructor(props) {
    super(props);

    this.openAddScheduleModal = this.openAddScheduleModal.bind(this);
    this.closeAddScheduleModal = this.closeAddScheduleModal.bind(this);
    this.openAddScheduleModalAndEditSchedule = this.openAddScheduleModalAndEditSchedule.bind(
      this
    );
    this.deleteLatestScheduleFromScheduleList = this.deleteLatestScheduleFromScheduleList.bind(
      this
    );
  }

  openAddScheduleModal() {
    this.props.scheduleEditorActions.setIsAddScheduleModalOpen(true);
  }

  closeAddScheduleModal() {
    this.props.scheduleEditorActions.setIsAddScheduleModalOpen(false);
  }

  openAddScheduleModalAndEditSchedule() {
    this.props.scheduleEditorActions.openAddScheduleModalAndEditSchedule();
  }

  deleteLatestScheduleFromScheduleList() {
    this.props.scheduleEditorActions.deleteLatestScheduleFromScheduleList();
  }

  render() {
    const wrapper = {
      padding: '5px',
      boxSizing: 'border-box',
      height: '100%'
    };

    const contentStyle = {
      boxSizing: 'border-box',
      height: 'calc(100% - 64px - 52px)'
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
              <div>
                <Button
                  variant="contained"
                  color="primary"
                  onClick={this.openAddScheduleModal}
                >
                  <AddIcon style={{ color: 'white', marginRight: '5px' }} />
                  Add Schedule
                </Button>
                <Button
                  variant="contained"
                  color="primary"
                  onClick={this.openAddScheduleModalAndEditSchedule}
                  style={{ marginLeft: '5px' }}
                >
                  <EditIcon style={{ color: 'white', marginRight: '5px' }} />
                  Edit Latest Schedule
                </Button>
                <Button
                  variant="contained"
                  color="primary"
                  onClick={this.deleteLatestScheduleFromScheduleList}
                  style={{ marginLeft: '5px' }}
                >
                  <DeleteIcon style={{ color: 'white', marginRight: '5px' }} />
                  Delete Latest Schedule
                </Button>
              </div>
            }
            title="Schedule"
            id="ScheduleCardHeader"
          />
          <CardContent style={contentStyle}>
            <ScheduleList />
          </CardContent>
          <CardActions>
            <ScheduleDownloadButton />
          </CardActions>
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
const mapState = state => ({
  isAddScheduleModalOpen: state.scheduleEditor.getIsAddScheduleModalOpen()
});
const mapDispatch = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export default connect(
  mapState,
  mapDispatch
)(ScheduleBox);
