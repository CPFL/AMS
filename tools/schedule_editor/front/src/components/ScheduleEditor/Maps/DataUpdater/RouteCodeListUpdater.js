import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

class RouteCodeListUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidMount() {
    if (this.props.routeCodeList) {
      this.props.initRouteCodeList(this.props.routeCodeList);
    }
  }

  componentDidUpdate() {
    if (this.props.routeCodeList) {
      this.props.updateRouteCodeList(this.props.routeCodeList);
    }
  }

  render() {
    return <div />;
  }
}
RouteCodeListUpdater.propTypes = {
  routeCodeList: PropTypes.array,
  initRouteCodeList: PropTypes.func,
  updateRouteCodeList: PropTypes.func
};
const mapState = state => ({
  routeCodeList: state.scheduleEditor.getRouteCodeList()
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(RouteCodeListUpdater);
