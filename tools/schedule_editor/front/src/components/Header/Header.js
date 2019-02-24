import React from 'react';
import { connect } from 'react-redux';
import { bindActionCreators } from 'redux';

import * as ScheduleEditorActions from '../../redux/Actions/ScheduleEditorActions';
import { AppBar, Button, Toolbar, Typography } from '@material-ui/core';
import InputIcon from '@material-ui/icons/Input';

import ImportDataModal from './ImportDataModal';
import PropTypes from 'prop-types';

class Header extends React.Component {
  constructor(props) {
    super(props);
    this.isWaypointAndLaneLoaderOpen = this.isWaypointAndLaneLoaderOpen.bind(
      this
    );
  }

  isWaypointAndLaneLoaderOpen() {
    this.props.scheduleEditorActions.setIsImportDataModalOpen(true);
  }

  render() {
    return (
      <div style={{ flexGrow: 1 }}>
        <AppBar position="static">
          <Toolbar>
            <Typography variant="h6" color="inherit" style={{ flexGrow: 1 }}>
              Schedule Editor
            </Typography>
            <Button
              style={{ color: 'white' }}
              color="default"
              onClick={this.isWaypointAndLaneLoaderOpen}
            >
              Import Map Data
              <InputIcon style={{ color: 'white', marginLeft: '5px' }} />
            </Button>
            <ImportDataModal />
          </Toolbar>
        </AppBar>
      </div>
    );
  }
}

Header.propTypes = {
  scheduleEditorActions: PropTypes.object
};
const mapState = () => ({});
const mapDispatch = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});

export default connect(
  mapState,
  mapDispatch
)(Header);
