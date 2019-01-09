import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';

import { decisionSectionRouteCodeSelector } from '../../../../../../redux/selectors/ScheduleEditorSelector';

class DecisionSectionRouteCodeUpdater extends React.Component {
  constructor(props) {
    super(props);
  }

  componentDidMount() {
    if (this.props.decisionSectionRouteCode) {
      this.props.initDecisionSectionRouteCode(
        this.props.decisionSectionRouteCode
      );
    }
  }

  componentDidUpdate() {
    if (this.props.decisionSectionRouteCode) {
      this.props.updateDecisionSectionRouteCode(
        this.props.decisionSectionRouteCode
      );
    }
  }

  render() {
    return <div />;
  }
}
DecisionSectionRouteCodeUpdater.propTypes = {
  decisionSectionRouteCode: PropTypes.number,
  initDecisionSectionRouteCode: PropTypes.func,
  updateDecisionSectionRouteCode: PropTypes.func
};

const mapState = state => ({
  decisionSectionRouteCode: decisionSectionRouteCodeSelector(state)
});
const mapDispatch = () => ({});
export default connect(
  mapState,
  mapDispatch
)(DecisionSectionRouteCodeUpdater);
