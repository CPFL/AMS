import React from 'react';
import PropTypes from 'prop-types';

import connect from 'react-redux/es/connect/connect';

import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import ListItemIcon from '@material-ui/core/ListItemIcon';
import ListItemText from '@material-ui/core/ListItemText';
import DoneIcon from '@material-ui/icons/Done';
import Button from '@material-ui/core/Button/Button';

class RouteList extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      listHeight: 0
    };
    this.deleteRouteCode = this.deleteRouteCode.bind(this);
  }

  deleteRouteCode(event) {
    console.log(event.target);
  }

  getItems() {
    const routeCodeList = this.props.routeCodeList;
    const resList = [];

    routeCodeList.forEach((item, index) => {
      resList.push(
        <ListItem button onClick={event => this.selectRouteCode(event, item)}>
          <ListItemIcon>
            <DoneIcon />
          </ListItemIcon>
          <ListItemText
            primary={
              <div style={{ wordBreak: 'break-all' }}>
                {item.routeCode}
                <Button
                  color="secondary"
                  onClick={this.deleteRouteCode}
                  value={index}
                  style={{marginLeft: 'auto'}}
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

RouteList.propTypes = {
  routeCodeList: PropTypes.array
};
const mapStateSelectEndPoint = state => ({
  routeCodeList: state.scheduleEditor.getRouteCodeList()
});
const mapDispatchSelectEndPoint = () => ({});
export default connect(
  mapStateSelectEndPoint,
  mapDispatchSelectEndPoint
)(RouteList);
