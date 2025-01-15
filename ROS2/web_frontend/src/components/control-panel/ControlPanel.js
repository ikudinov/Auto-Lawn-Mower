import React from "react";
import "./ControlPanel.css";
import Joystick from "../joystick/Joystick";
import PowerMeter from "../power-meter/PowerMeter";
import { calcJoystickTop, calcJoystickRight } from "./helper";

export default function ControlPanel({ leftPower, rightPower, trimmerEnabled }) {
  const joystickTop = calcJoystickTop(leftPower, rightPower);
  const joystickRight = calcJoystickRight(leftPower, rightPower);

  return (
    <div className="control-panel">
      <div className="control-panel__joystick">
        <Joystick top={joystickTop} right={joystickRight} />
      </div>
      <div className="control-panel__group">
        <PowerMeter value={leftPower} label="Левый" />
        <PowerMeter value={rightPower} label="Правый" />
      </div>
      <PowerMeter value={trimmerEnabled ? 100 : 0} label="Триммер" />
    </div>
  );
}
