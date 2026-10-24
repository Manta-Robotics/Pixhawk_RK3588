import {useLayoutEffect} from 'react';
import {createRoot} from 'react-dom/client';
import {io} from 'socket.io-client';
import {ConsoleLayout} from './ConsoleLayout';
import {mountConsole} from './console-controller';
import './manta-app-transport';
import './offline-satellite-map';
import './uwb-display-smoother';
import './uwb-raw-display';
import './console.css';

window.io=io;
function App(){
  useLayoutEffect(mountConsole,[]);
  return <ConsoleLayout/>;
}
// Keep the original body-level hierarchy: no new layout wrapper or CSS reset.
createRoot(document.body).render(<App/>);
