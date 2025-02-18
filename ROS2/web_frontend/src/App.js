import React, { useState } from "react";
import "material-icons/iconfont/material-icons.css";
import "./App.css";
import ControlPanel from "./features/control-panel/ControlPanel";

const FUNC = {
  MANUAL: 1,
  AUTO: 2,
};

function App() {
  const [func, setFunc] = useState(FUNC.MANUAL);

  const handleManualClick = () => setFunc(FUNC.MANUAL);
  const handleAutoClick = () => setFunc(FUNC.AUTO);
  const handleFullscreen = () => {
    document
      .getElementById("root")
      .requestFullscreen()
      .catch(() => {});
  };

  return (
    <div className="App">
      <div className="control-buttons">
        <button
          className={`control-button ${
            func === FUNC.MANUAL ? "control-button--selected" : ""
          }`}
          onClick={handleManualClick}
        >
          РУЧНОЙ
        </button>
        <button
          className={`control-button ${
            func === FUNC.AUTO ? "control-button--selected" : ""
          }`}
          onClick={handleAutoClick}
        >
          АВТОМАТ
        </button>
        <button
          className="fullscreen-button"
          onClick={handleFullscreen}
          id="fullscreen-button"
        >
          <span className="material-icons">fullscreen</span>
        </button>
      </div>
      {func === FUNC.MANUAL && <ControlPanel />}
      {func === FUNC.AUTO && (
        <div className="not-implemented">Пока не реализовано</div>
      )}
    </div>
  );
}

export default App;
