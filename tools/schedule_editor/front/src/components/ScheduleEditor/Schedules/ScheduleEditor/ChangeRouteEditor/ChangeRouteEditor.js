import React from 'react';
import { bindActionCreators } from 'redux';
import connect from 'react-redux/es/connect/connect';

import Stepper from '@material-ui/core/Stepper';
import Step from '@material-ui/core/Step';
import StepLabel from '@material-ui/core/StepLabel';

import PropTypes from 'prop-types';
import * as ScheduleEditorActions from '../../../../../redux/Actions/ScheduleEditorActions';

//import { SelectStartPoint } from './CreateChangeRouteProcess';
import SelectRouteCodeAfterChangeRoute from './SelectRouteCodeAfterChangeRoute';

class ChangeRouteEditor extends React.Component {
  constructor(props) {
    super(props);

    this.steps = [
      {
        name: 'select Route Code After Change Route',
        component: <SelectRouteCodeAfterChangeRoute />
      },
      {
        name: 'Select Decision Section End Point ',
        component: 'Test'
      },
      {
        name: 'Result',
        component: 'Test'
      }
    ];
  }

  getChangeRouteSteps() {
    return this.steps[this.props.changeRouteActiveStep].component;
  }

  render() {
    return (
      <div>
        <Stepper activeStep={this.props.changeRouteActiveStep}>
          {this.steps.map(step => {
            return (
              <Step key={step.name}>
                <StepLabel>{step.name}</StepLabel>
              </Step>
            );
          })}
        </Stepper>
        <div>{this.getChangeRouteSteps()}</div>
      </div>
    );
  }
}
ChangeRouteEditor.propTypes = {
  changeRouteActiveStep: PropTypes.number,
  scheduleEditorActions: PropTypes.object
};
const mapStateSelectRouteCode = state => ({
  changeRouteActiveStep: state.scheduleEditor.getChangeRouteActiveStep()
});
const mapDispatchSelectRouteCode = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export default connect(
  mapStateSelectRouteCode,
  mapDispatchSelectRouteCode
)(ChangeRouteEditor);
