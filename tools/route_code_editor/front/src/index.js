import {AppContainer} from 'react-hot-loader';
import React from 'react';
import {render} from 'react-dom';
import App from './App';
import './style/style.css';
import './style/route_code_editor.css';

const rootEl = document.getElementById('root');
render(
    <AppContainer>
        <App/>
    </AppContainer>,
    rootEl
);
