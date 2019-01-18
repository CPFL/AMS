import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

import { selectScheduleDisplayMainViewerSelector } from '../../../../redux/selectors/ScheduleEditorSelector';

class SelectRouteCodeDisplayMainViewerUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidUpdate() {
    const selectScheduleDisplayMainViewer = this.props
      .selectScheduleDisplayMainViewer;
    if (
      selectScheduleDisplayMainViewer.startPoint != null &&
      selectScheduleDisplayMainViewer.laneList != null &&
      selectScheduleDisplayMainViewer.endPoint != null
    ) {
      this.props.updateSelectScheduleDisplayMainViewer(
        selectScheduleDisplayMainViewer
      );
    }
  }

  render() {
    return <div />;
  }
}

SelectRouteCodeDisplayMainViewerUpdater.propTypes = {
  selectScheduleDisplayMainViewer: PropTypes.object,
  updateSelectScheduleDisplayMainViewer: PropTypes.func
};
const mapState = state => ({
  selectScheduleDisplayMainViewer: selectScheduleDisplayMainViewerSelector(
    state
  )
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(SelectRouteCodeDisplayMainViewerUpdater);
