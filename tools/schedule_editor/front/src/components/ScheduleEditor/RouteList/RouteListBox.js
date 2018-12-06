import React from 'react';
import {connect} from "react-redux";

import Card from '@material-ui/core/Card';
import CardHeader from '@material-ui/core/CardHeader';
import CardContent from '@material-ui/core/CardContent';
import Button from '@material-ui/core/Button';
import Dialog from '@material-ui/core/Dialog';
import AppBar from '@material-ui/core/AppBar';
import Toolbar from '@material-ui/core/Toolbar';
import IconButton from '@material-ui/core/IconButton';
import CloseIcon from '@material-ui/icons/Close';
import Typography from '@material-ui/core/Typography';

import RouteList from './RouteList';
import RouteCodeEditor from '../RouteCodeEditor/RouteCodeEditor';

class RouteListBox extends React.Component {

  constructor(props) {
    super(props);
    this.state = {
      listHeight: 0,
      isOpen: false
    };

    this.routeCodeEditorModalOpen = this.routeCodeEditorModalOpen.bind(this);
    this.closeModal = this.closeModal.bind(this);
    this.resize = this.resize.bind(this);
  }

  componentDidMount() {
    this.setState({listHeight: document.getElementById("RouteListCard").clientHeight - document.getElementById("RouteListCardHeader").clientHeight});
    document.getElementById("RouteListCard").addEventListener('resize', this.resize, false);
  }

  resize() {
    this.setState({listHeight: document.getElementById("RouteListCard").clientHeight - document.getElementById("RouteListCardHeader").clientHeight});
  }

  routeCodeEditorModalOpen(){
    this.setState({isOpen: true});
  }

  closeModal(){
    this.setState({isOpen: false});
  }

  render() {
    const cardContentHeight = this.state.listHeight;

    const wrapper = {
      padding: '5px',
      boxSizing: 'border-box',
      height: '100%'
    };

    const contentStyle = {
      boxSizing: 'border-box',
      height: cardContentHeight
    };

    const modalHeader = {
      flexGrow: 1,
      height: '40px'
    };

    const modalContent = {
      position: 'absolute',
      top: '40px',
      right: 0,
      bottom: 0,
      left: 0
    };

    return (
      <div style={wrapper}>
        <Card style={{height: '100%'}}
              id="RouteListCard"
        >
          <CardHeader
            action={
              <Button variant="outlined" onClick={this.routeCodeEditorModalOpen}>
                Add Route
              </Button>
            }
            title="Route List"
            id="RouteListCardHeader"
          />
          <CardContent style={contentStyle}>
            <RouteList/>
          </CardContent>
        </Card>
        <Dialog
          fullScreen
          open={this.state.isOpen}
          onClose={this.closeModal}
        >
          <AppBar style={modalHeader}>
            <Toolbar style={{minHeight: '40px'}}>
              <IconButton color="inherit" onClick={this.closeModal} aria-label="Close">
                <CloseIcon />
              </IconButton>
              <Typography variant="h6" color="inherit">
                Route Code Editor
              </Typography>
            </Toolbar>
          </AppBar>
          <div style={modalContent}>
            <RouteCodeEditor/>
          </div>

        </Dialog>
      </div>

    );
  }
}

const mapState = () => ({});

const mapDispatch = () => ({});

export default connect(mapState, mapDispatch)(RouteListBox);

