import React from 'react';
import PropTypes from 'prop-types';

import { connect } from 'react-redux';
import { bindActionCreators } from 'redux';

import Card from '@material-ui/core/Card';
import CardHeader from '@material-ui/core/CardHeader';
import CardContent from '@material-ui/core/CardContent';
import CardActions from '@material-ui/core/CardActions';
import Button from '@material-ui/core/Button';
import Typography from '@material-ui/core/Typography/Typography';

import * as ScheduleEditorActions from '../../../../../redux/Actions/ScheduleEditorActions';
import { changeRouteSteps } from '../../../../../model/Redux/Page/ScheduleEditor';

class SelectDecisionSectionRouteCode extends React.Component {
  constructor(props) {
    super(props);
    this.confirm = this.confirm.bind(this);
    this.back = this.back.bind(this);
  }

  confirm() {
    if (this.props.decisionSectionRouteCode.routeCode) {
      this.props.scheduleEditorActions.setChangeRouteActiveStepNext();
    } else {
      alert('Decision Section End Point is not selected!');
    }
  }

  back() {
    this.props.scheduleEditorActions.setChangeRouteActiveStepPrevious();
  }

  render() {
    return (
      <Card shadow={0} style={{ width: '100%', minHeight: '100px' }}>
        <CardHeader
          title={changeRouteSteps[this.props.changeRouteActiveStep].name}
        />
        <CardContent>
          <Typography variant="subtitle1">
            <strong>
              Decision Section Route Code:
              {this.props.decisionSectionRouteCode.routeCode}
            </strong>
          </Typography>
        </CardContent>
        <CardActions>
          <div style={{ marginLeft: 'auto' }}>
            <Button onClick={this.back}>Back</Button>
            <Button
              color="primary"
              onClick={this.confirm}
              style={{ marginLeft: '5px' }}
            >
              Confirm
            </Button>
          </div>
        </CardActions>
      </Card>
    );
  }
}

SelectDecisionSectionRouteCode.propTypes = {
  changeRouteActiveStep: PropTypes.number,
  decisionSectionRouteCode: PropTypes.string,
  scheduleEditorActions: PropTypes.object
};

const mapState = state => ({
  changeRouteActiveStep: state.scheduleEditor.getChangeRouteActiveStep(),
  decisionSectionRouteCode: state.scheduleEditor.getDecisionSectionRouteCode()
});
const mapDispatch = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export default connect(
  mapState,
  mapDispatch
)(SelectDecisionSectionRouteCode);
