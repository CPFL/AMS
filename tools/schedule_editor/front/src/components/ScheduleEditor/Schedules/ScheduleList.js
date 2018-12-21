import React from 'react';

import List from '@material-ui/core/List';
import PropTypes from 'prop-types';
import connect from 'react-redux/es/connect/connect';
import ListItem from '@material-ui/core/ListItem/ListItem';
import ListItemIcon from '@material-ui/core/ListItemIcon/ListItemIcon';
import DoneIcon from '@material-ui/icons/Done';
import ListItemText from '@material-ui/core/ListItemText/ListItemText';
import Typography from '@material-ui/core/Typography';
import { bindActionCreators } from 'redux';
import * as ScheduleEditorActions from '../../../redux/Actions/ScheduleEditorActions';

class ScheduleList extends React.Component {
  constructor(props) {
    super(props);
  }

  getScheduleList() {
    let scheduleList = this.props.scheduleList;

    const resList = [];
    for (const schedule of scheduleList) {
      let sendEngageText = '';
      if (schedule.checkedSendEngage) {
        sendEngageText = (
          <Typography variant="subheading">
            Send Engage: {schedule.waitTime} Second
          </Typography>
        );
      }
      resList.push(
        <ListItem
          button
          onClick={event => this.selectSchedule(event, schedule)}
        >
          <ListItemIcon>
            <DoneIcon />
          </ListItemIcon>
          <ListItemText
            primary={
              <div>
                <Typography variant="subheading">
                  <div style={{ wordBreak: 'break-all' }}>
                    Send Lane Array: {schedule.routeCode}
                  </div>
                </Typography>
                {sendEngageText}
              </div>
            }
          />
        </ListItem>
      );
    }
    return resList;
  }

  selectSchedule(event, schedule) {
    this.props.scheduleEditorActions.setSelectScheduleDisplayMainViewer(schedule);
    console.log(schedule);
  }

  render() {
    const routeListStyle = {
      overflowY: 'auto',
      height: '100%'
    };

    return <List style={routeListStyle}>{this.getScheduleList()}</List>;
  }
}
ScheduleList.propTypes = {
  scheduleList: PropTypes.array,
  scheduleEditorActions: PropTypes.object
};

const mapState = state => ({
  scheduleList: state.scheduleEditor.getScheduleList()
});
const mapDispatch = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export default connect(
  mapState,
  mapDispatch
)(ScheduleList);
