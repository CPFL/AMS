import React from 'react';
import PropTypes from 'prop-types';

import Button from '@material-ui/core/Button';
import connect from 'react-redux/es/connect/connect';

class ScheduleDownloadButton extends React.Component {
  constructor(props) {
    super(props);

    this.downloadSchedule = this.downloadSchedule.bind(this);
  }

  downloadSchedule() {
    const outputSchedules = [];
    for (const schedule of this.props.scheduleList) {
      outputSchedules.push({
        name: 'send_lane_array',
        route_code: schedule.routeCode
      });
      if (schedule.checkedSendEngage) {
        outputSchedules.push({
          name: 'send_engage'
        });
        if (schedule.waitTime > 0) {
          outputSchedules.push({
            name: 'wait_time',
            second: schedule.waitTime
          });
        }
      }
    }
    this.handleDownload(outputSchedules);
  }

  handleDownload(content) {
    const blob = new Blob([JSON.stringify(content, null, 2)], {
      type: 'application/json'
    });

    document.getElementById('download').href = window.URL.createObjectURL(blob);
  }

  render() {
    return (
      <div style={{ marginLeft: 'auto' }}>
        <Button
          id="download"
          download="schedule.json"
          href="#"
          color="primary"
          onClick={this.downloadSchedule}
        >
          Download Schedule
        </Button>
      </div>
    );
  }
}

ScheduleDownloadButton.propTypes = {
  scheduleList: PropTypes.array
};
const mapStateSelect = state => ({
  scheduleList: state.scheduleEditor.getScheduleList()
});
const mapDispatch = () => ({});
export default connect(
  mapStateSelect,
  mapDispatch
)(ScheduleDownloadButton);
