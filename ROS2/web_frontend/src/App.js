import React, { useState } from 'react';
import './App.css';
import ControlPanel from './features/control-panel/ControlPanel';

const FUNC = {
  MANUAL: 1,
  AUTO: 2,
}

function App() {
  const [func, setFunc] = useState(FUNC.MANUAL);

  const handleManualClick = () => setFunc(FUNC.MANUAL);
  const handleAutoClick = () => setFunc(FUNC.AUTO);

  return (
    <div className="App">
      <div className="control-buttons">
        <button
          className={`control-button ${func === FUNC.MANUAL ? 'control-button--selected' : ''}`}
          onClick={handleManualClick}
        >
          РУЧНОЙ
        </button>
        <button
          className={`control-button ${func === FUNC.AUTO ? 'control-button--selected' : ''}`}
          onClick={handleAutoClick}
        >
          АВТОМАТ
        </button>
      </div>
      {func === FUNC.MANUAL && <ControlPanel />}
      {func === FUNC.AUTO && (
        <div className='not-implemented'>Пока не реализовано</div>
      )}
      </div>
  );
}

export default App;
