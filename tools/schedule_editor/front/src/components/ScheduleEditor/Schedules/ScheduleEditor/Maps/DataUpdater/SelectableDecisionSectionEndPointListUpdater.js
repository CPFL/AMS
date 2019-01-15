import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

import { selectableDecisionSectionEndPointListSelector } from '../../../../../../redux/selectors/ScheduleEditorSelector';

class SelectableDecisionSectionEndPointListUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidMount() {
    if (this.props.selectableDecisionSectionEndPointList) {
      this.props.initSelectableDecisionSectionEndPointList(
        this.props.selectableDecisionSectionEndPointList
      );
    }
  }

  componentDidUpdate() {
    if (this.props.selectableDecisionSectionEndPointList) {
      this.props.setSelectableDecisionSectionEndPointList(
        this.props.selectableDecisionSectionEndPointList
      );
    }
  }

  render() {
    return <div />;
  }
}
SelectableDecisionSectionEndPointListUpdater.propTypes = {
  selectableDecisionSectionEndPointList: PropTypes.number,
  initSelectableDecisionSectionEndPointList: PropTypes.func,
  setSelectableDecisionSectionEndPointList: PropTypes.func
};

const mapState = state => ({
  selectableDecisionSectionEndPointList: selectableDecisionSectionEndPointListSelector(state)
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(SelectableDecisionSectionEndPointListUpdater);
