import React from 'react';
import connect from 'react-redux/es/connect/connect';

import Stepper from '@material-ui/core/Stepper';
import Step from '@material-ui/core/Step';
import StepLabel from '@material-ui/core/StepLabel';

import PropTypes from 'prop-types';

import SelectRouteCodeAfterChangeRoute from './SelectRouteCodeAfterChangeRoute';
import SelectDecisionSectionEndPoint from './SelectDecisionSectionEndPoint';
import Result from './Result';

class ChangeRouteEditor extends React.Component {
  constructor(props) {
    super(props);

    this.steps = [
      {
        name: 'select Route Code After Change Route',
        component: <SelectRouteCodeAfterChangeRoute />
      },
      {
        name: 'Select Decision Section Route Code',
        component: <SelectDecisionSectionEndPoint />
      },
      {
        name: 'Result',
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
