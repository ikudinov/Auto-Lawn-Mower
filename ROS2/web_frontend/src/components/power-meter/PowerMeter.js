import React from "react";
import "./PowerMeter.css";

const BAR_HEIGHT = 300;

export default function PowerMeter({ label, value }) {
  const height = Math.abs((BAR_HEIGHT * value) / 200);
  const bottom = value > 0 ? (BAR_HEIGHT * value) / 200 : 0;
  const text = Math.abs(value) >= 20 ? `${Math.abs(value)}%` : "";

  return (
    <div className="power-meter">
      <div className="power-meter__label">{label}</div>
      <div className="power-meter__bar">
        <div
          className="power-meter__bar-line"
          style={{ height: `${height}px`, bottom: `${bottom}px` }}
        >
          {text}
        </div>
      </div>
    </div>
  );
}
