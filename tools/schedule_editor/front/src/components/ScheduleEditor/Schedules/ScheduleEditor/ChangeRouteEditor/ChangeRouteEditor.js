import React from 'react';
import connect from 'react-redux/es/connect/connect';

import Stepper from '@material-ui/core/Stepper';
import Step from '@material-ui/core/Step';
import StepLabel from '@material-ui/core/StepLabel';

import PropTypes from 'prop-types';

import SelectRouteCodeAfterChangeRoute from './SelectRouteCodeAfterChangeRoute';
import SelectDecisionSectionRouteCode from './SelectDecisionSectionRouteCode';
import Result from './Result';

import { changeRouteSteps } from '../../../../../model/Redux/Page/ScheduleEditor';

class ChangeRouteEditor extends React.Component {
  constructor(props) {
    super(props);

    this.steps = [
      {
        component: <SelectRouteCodeAfterChangeRoute />
      },
      {
        component: <SelectDecisionSectionRouteCode />
      },
      {
        component: <Result />
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
          {changeRouteSteps.map(step => {
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
  changeRouteActiveStep: PropTypes.number
};
const mapStateSelectRouteCode = state => ({
  changeRouteActiveStep: state.scheduleEditor.getChangeRouteActiveStep()
});
const mapDispatchSelectRouteCode = () => ({});
export default connect(
  mapStateSelectRouteCode,
  mapDispatchSelectRouteCode
)(ChangeRouteEditor);
