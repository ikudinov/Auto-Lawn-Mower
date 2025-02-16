import React from "react";
import "./ControlPanel.css";
import Joystick from "../../components/joystick/Joystick";
import PowerMeter from "../../components/power-meter/PowerMeter";
import { calcJoystickTop, calcJoystickRight } from "./helper";
import { useGetMessagesQuery } from "./controlPanelApi";

export default function ControlPanel() {
  const { currentData } = useGetMessagesQuery();

  const motors =
    currentData && currentData.motors
      ? currentData.motors
      : { left: 0, right: 0, trimmer: 0 };

  const joystickTop = calcJoystickTop(motors.left, motors.right);
  const joystickRight = calcJoystickRight(motors.left, motors.right);

  return (
    <div className="control-panel">
      <div className="control-panel__joystick">
        <Joystick top={joystickTop} right={joystickRight} />
      </div>
      <div className="control-panel__group">
        <PowerMeter value={motors.left} label="Левый" />
        <PowerMeter value={motors.right} label="Правый" />
      </div>
      <PowerMeter value={motors.trimmer ? 100 : 0} label="Триммер" />
    </div>
  );
}
