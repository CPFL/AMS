import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

import { currentEditChangeRouteListSelector } from '../../../../../../redux/selectors/ScheduleEditorSelector';

class CurrentEditChangeRouteListUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidMount() {
    if (this.props.currentEditChangeRouteList) {
      this.props.initCurrentEditChangeRouteList(
        this.props.currentEditChangeRouteList
      );
    }
  }

  componentDidUpdate() {
    if (this.props.currentEditChangeRouteList) {
      this.props.updateCurrentEditChangeRouteList(
        this.props.currentEditChangeRouteList
      );
    }
  }

  render() {
    return <div />;
  }
}

CurrentEditChangeRouteListUpdater.propTypes = {
  currentEditChangeRouteList: PropTypes.number,
  initCurrentEditChangeRouteList: PropTypes.func,
  updateCurrentEditChangeRouteList: PropTypes.func
};

const mapState = state => ({
  currentEditChangeRouteList: currentEditChangeRouteListSelector(state)
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(CurrentEditChangeRouteListUpdater);
