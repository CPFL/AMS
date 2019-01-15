import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

import { selectedDisplayRouteMainViewerSelector } from '../../../../redux/selectors/ScheduleEditorSelector';

class SelectedDisplayRouteMainViewerUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidUpdate() {
    const selectedDisplayRouteMainViewer = this.props
      .selectedDisplayRouteMainViewer;
    if (selectedDisplayRouteMainViewer) {
      this.props.updateSelectedDisplayRouteMainViewer(
        selectedDisplayRouteMainViewer
      );
    }
  }

  render() {
    return <div />;
  }
}

SelectedDisplayRouteMainViewerUpdater.propTypes = {
  selectedDisplayRouteMainViewer: PropTypes.object,
  updateSelectedDisplayRouteMainViewer: PropTypes.func
};
const mapState = state => ({
  selectedDisplayRouteMainViewer: selectedDisplayRouteMainViewerSelector(state)
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(SelectedDisplayRouteMainViewerUpdater);
