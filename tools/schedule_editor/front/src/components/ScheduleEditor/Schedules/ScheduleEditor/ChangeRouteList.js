import React from 'react';
import PropTypes from 'prop-types';

import connect from 'react-redux/es/connect/connect';

import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import ListItemIcon from '@material-ui/core/ListItemIcon';
import ListItemText from '@material-ui/core/ListItemText';
import DoneIcon from '@material-ui/icons/Done';

class ChangeRouteList extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      listHeight: 0
    };
    this.selectRouteCode.bind(this);
  }

  getItems() {
    let currentEditChangeRouoteList = this.props.currentEditChangeRouoteList;
    console.log(currentEditChangeRouoteList);
    currentEditChangeRouoteList = ['test', 'test2', 'test4'];

    const resList = [];
    for (const item of currentEditChangeRouoteList) {
      resList.push(
        <ListItem button onClick={event => this.selectRouteCode(event, item)}>
          <ListItemIcon>
            <DoneIcon />
          </ListItemIcon>
          <ListItemText
            primary={<div style={{ wordBreak: 'break-all' }}>{item}</div>}
          />
        </ListItem>
      );
    }
    return resList;
  }

  selectRouteCode(event, item) {
    console.log(item);
  }

  render() {
    const routeListStyle = {
      overflowY: 'auto',
      height: '100%'
    };

    return <List style={routeListStyle}>{this.getItems()}</List>;
  }
}
ChangeRouteList.propTypes = {
  currentEditChangeRouoteList: PropTypes.array
};
const mapStateSelectEndPoint = state => ({
  currentEditChangeRouoteList: state.scheduleEditor.getCurrentEditChangeRouoteList()
});
const mapDispatchSelectEndPoint = () => ({});
export default connect(
  mapStateSelectEndPoint,
  mapDispatchSelectEndPoint
)(ChangeRouteList);
