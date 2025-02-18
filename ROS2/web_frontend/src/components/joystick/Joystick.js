import React from "react";
import "./Joystick.css";

const JOYSTICK_SIZE = 250;
const JOYSTICK_POINTER_SIZE = 40;

export default function Joystick({ top, right }) {
  const marginBottom = ((JOYSTICK_SIZE - JOYSTICK_POINTER_SIZE) * top) / 200;
  const marginRight = (-right * (JOYSTICK_SIZE - JOYSTICK_POINTER_SIZE)) / 200;

  return (
    <div className="joystick">
      <div className="joystick__circle" />
      <div
        className="joystick__pointer"
        style={{
          marginBottom: `${marginBottom}px`,
          marginRight: `${marginRight}px`,
        }}
      />
    </div>
  );
}
