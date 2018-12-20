import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

class ScheduleListUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidMount() {
    if (this.props.scheduleList !== undefined) {
      this.props.initScheduleList(this.props.scheduleList);
    }
  }

  render() {
    return <div />;
  }
}

ScheduleListUpdater.propTypes = {
  scheduleList: PropTypes.array,
  initScheduleList: PropTypes.func
};
const mapState = state => ({
  scheduleList: state.scheduleEditor.getScheduleList()
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(ScheduleListUpdater);
