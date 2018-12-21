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
    this.handleDownload(JSON.stringify(outputSchedules, null, 2));
  }

  handleDownload(content) {
    const blob = new Blob([content], { type: 'text/plain' });

    document.getElementById('download').href = window.URL.createObjectURL(blob);
  }

  render() {
    return (
      <div style={{ marginLeft: 'auto' }}>
        <Button
          id="download"
          download="schedule.txt"
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
