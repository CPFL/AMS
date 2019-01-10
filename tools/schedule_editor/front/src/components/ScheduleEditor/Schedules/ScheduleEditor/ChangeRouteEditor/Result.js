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

class SelectDecisionSectionEndPoint extends React.Component {
  constructor(props) {
    super(props);
    this.save = this.save.bind(this);
    this.back = this.back.bind(this);
  }

  save() {
    this.props.scheduleEditorActions.saveChangeRoute(
      this.props.routeCodeAfterChangeRoute,
      this.props.decisionSectionRouteCode
    );
  }

  back() {
    this.props.scheduleEditorActions.setChangeRouteActiveStepPrevious();
  }

  render() {
    return (
      <Card shadow={0} style={{ width: '100%', minHeight: '100px' }}>
        <CardHeader title="Select Decision Section End Point" />
        <CardContent>
          <Typography variant="subtitle1">
            <strong>
              Route Code After Changed:
              {this.props.routeCodeAfterChangeRoute.routeCode}
            </strong>
          </Typography>
          <Typography variant="subtitle1">
            <strong>
              Decision Section End Point ID:
              {this.props.decisionSectionRouteCode.routeCode}
            </strong>
          </Typography>
        </CardContent>
        <CardActions>
          <div style={{ marginLeft: 'auto' }}>
            <Button onClick={this.back}>Back</Button>
            <Button
              color="primary"
              onClick={this.save}
              style={{ marginLeft: '5px' }}
            >
              Save
            </Button>
          </div>
        </CardActions>
      </Card>
    );
  }
}

SelectDecisionSectionEndPoint.propTypes = {
  routeCodeAfterChangeRoute: PropTypes.object,
  decisionSectionRouteCode: PropTypes.string,
  scheduleEditorActions: PropTypes.object
};

const mapState = state => ({
  routeCodeAfterChangeRoute: state.scheduleEditor.getRouteCodeAfterChangeRoute(),
  decisionSectionRouteCode: state.scheduleEditor.getDecisionSectionRouteCode()
});
const mapDispatch = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export default connect(
  mapState,
  mapDispatch
)(SelectDecisionSectionEndPoint);
