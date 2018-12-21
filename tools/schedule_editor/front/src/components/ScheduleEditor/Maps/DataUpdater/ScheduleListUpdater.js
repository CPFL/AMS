import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

class RouteCodeListUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidMount() {
    if (this.props.scheduleList) {
      this.props.initScheduleList(this.props.scheduleList);
    }
  }

  componentDidUpdate() {
    if (this.props.scheduleList) {
      this.props.updateScheduleList(this.props.scheduleList);
    }
  }

  render() {
    return <div />;
  }
}
RouteCodeListUpdater.propTypes = {
  scheduleList: PropTypes.array,
  initScheduleList: PropTypes.func,
  updateScheduleList: PropTypes.func
};
const mapState = state => ({
  scheduleList: state.scheduleEditor.getScheduleList()
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(RouteCodeListUpdater);
