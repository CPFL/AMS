import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

import { selectRouteCodeDisplayMainViewerSelector } from '../../../../redux/selectors/ScheduleEditorSelector';

class SelectRouteCodeDisplayMainViewerUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidUpdate() {
    const selectRouteCodeDisplayMainViewer = this.props.selectRouteCodeDisplayMainViewer;
    if (
      selectRouteCodeDisplayMainViewer.startPoint != null &&
      selectRouteCodeDisplayMainViewer.laneList != null &&
      selectRouteCodeDisplayMainViewer.endPoint != null
    ) {
      this.props.updateSelectRouteCodeDisplayMainViewer(
        selectRouteCodeDisplayMainViewer
      );
    }
  }

  render() {
    return <div />;
  }
}

SelectRouteCodeDisplayMainViewerUpdater.propTypes = {
  selectRouteCodeDisplayMainViewer: PropTypes.object,
  updateSelectRouteCodeDisplayMainViewer: PropTypes.func
};
const mapState = state => ({
  selectRouteCodeDisplayMainViewer: selectRouteCodeDisplayMainViewerSelector(
    state
  )
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(SelectRouteCodeDisplayMainViewerUpdater);
