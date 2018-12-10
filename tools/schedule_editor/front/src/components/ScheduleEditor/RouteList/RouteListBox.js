import React from 'react';

import Card from '@material-ui/core/Card';
import CardHeader from '@material-ui/core/CardHeader';
import CardContent from '@material-ui/core/CardContent';
import Button from '@material-ui/core/Button';
import Dialog from '@material-ui/core/Dialog';

import RouteList from './RouteList';
import RouteCodeEditor from '../RouteCodeEditor/RouteCodeEditor';

export default class RouteListBox extends React.Component {

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

  routeCodeEditorModalOpen() {
    this.setState({isOpen: true});
  }

  closeModal() {
    this.setState({isOpen: false});
  }

  render() {
    const cardContentHeight = this.state.listHeight;

    const wrapper = {
      padding: '5px',
      boxSizing: 'border-box',
      height: '100%'
    };

    const CardStyle = {
      height: '100%'
    };

    const contentStyle = {
      boxSizing: 'border-box',
      height: cardContentHeight
    };


    const modalContent = {
      height: window.innerHeight * 0.9
    };

    return (
      <div style={wrapper}>
        <Card style={CardStyle}
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
          open={this.state.isOpen}
          onClose={this.closeModal}
          fullWidth={true}
          maxWidth='xl'
        >
          <div style={modalContent}>
            <RouteCodeEditor/>
          </div>

        </Dialog>
      </div>

    );
  }
}

