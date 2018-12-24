import React from 'react';
import PropTypes from 'prop-types';

import connect from 'react-redux/es/connect/connect';

import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import ListItemIcon from '@material-ui/core/ListItemIcon';
import ListItemText from '@material-ui/core/ListItemText';
import DoneIcon from '@material-ui/icons/Done';
import Button from '@material-ui/core/Button/Button';
import { bindActionCreators } from 'redux';
import * as ScheduleEditorActions from '../../../redux/Actions/ScheduleEditorActions';

class RouteList extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      listHeight: 0
    };
  }

  selectRouteCode(event, routeCode) {
    this.props.scheduleEditorActions.setSelectRouteCodeDisplayMainViewer(
      routeCode
    );
  }

  deleteRouteCode(event, index) {
    this.props.scheduleEditorActions.deleteRouteCodeFromRouteCodeListByIndex(
      index
    );
  }

  getItems() {
    const routeCodeList = this.props.routeCodeList;
    const resList = [];

    routeCodeList.forEach((routeCode, index) => {
      resList.push(
        <ListItem>
          <ListItemIcon>
            <DoneIcon />
          </ListItemIcon>
          <ListItemText
            primary={
              <div style={{ wordBreak: 'break-all' }}>
                <Button
                  color="default"
                  onClick={event => this.selectRouteCode(event, routeCode)}
                  value={index}
                >
                  {routeCode.routeCode}
                </Button>
                <Button
                  color="secondary"
                  onClick={event => this.deleteRouteCode(event, index)}
                  style={{ marginLeft: 'auto' }}
                >
                  Delete
                </Button>
              </div>
            }
          />
        </ListItem>
      );
    });
    return resList;
  }

  render() {
    const routeListStyle = {
      overflowY: 'auto',
      height: '100%'
    };

    return <List style={routeListStyle}>{this.getItems()}</List>;
  }
}

RouteList.propTypes = {
  routeCodeList: PropTypes.array,
  scheduleEditorActions: PropTypes.object
};
const mapStateSelectEndPoint = state => ({
  routeCodeList: state.scheduleEditor.getRouteCodeList()
});
const mapDispatchSelectEndPoint = dispatch => ({
  scheduleEditorActions: bindActionCreators(ScheduleEditorActions, dispatch)
});
export default connect(
  mapStateSelectEndPoint,
  mapDispatchSelectEndPoint
)(RouteList);
