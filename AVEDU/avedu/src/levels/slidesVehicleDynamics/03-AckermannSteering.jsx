// src/levels/slidesVehicleDynamics/03-AckermannSteering.jsx
import React, { useState, useRef, useEffect } from "react";

export const meta = {
  id: "vd-ackermann",
  title: "Ackermann Steering Geometry",
  order: 3,
  objectiveCode: "vd-slide-ackermann",
};

function AckermannVisualization({ wheelbase, trackWidth, innerAngle }) {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const w = canvas.width;
    const h = canvas.height;

    // Clear canvas
    ctx.fillStyle = "#0a0e1a";
    ctx.fillRect(0, 0, w, h);

    const scale = 80;
    const centerX = w / 2;
    const centerY = h / 2 + 50;

    // Calculate outer wheel angle using Ackermann formula
    const innerRad = (innerAngle * Math.PI) / 180;
    const L = wheelbase;
    const t = trackWidth;

    // Ackermann formula: cot(δo) = cot(δi) + t/L
    const cotInner = 1 / Math.tan(innerRad);
    const cotOuter = cotInner + t / L;
    const outerRad = Math.atan(1 / cotOuter);
    const outerAngle = (outerRad * 180) / Math.PI;

    // Calculate ICR position
    let icrX = centerX;
    let icrY = centerY;
    let R = Infinity;

    if (Math.abs(innerRad) > 0.001) {
      R = L / Math.tan(Math.abs(innerRad));
      const sign = innerAngle > 0 ? -1 : 1;
      icrX = centerX + sign * R * scale;
      icrY = centerY + (wheelbase * scale / 2);
    }

    // Draw vehicle
    ctx.save();
    ctx.translate(centerX, centerY);

    // Vehicle body
    ctx.strokeStyle = "#7df9ff";
    ctx.lineWidth = 3;
    ctx.strokeRect(
      -trackWidth * scale / 2,
      -wheelbase * scale / 2,
      trackWidth * scale,
      wheelbase * scale
    );

    const frontY = -wheelbase * scale / 2;
    const rearY = wheelbase * scale / 2;
    const leftX = -trackWidth * scale / 2;
    const rightX = trackWidth * scale / 2;

    // Determine which wheel is inner/outer based on turn direction
    const isLeftTurn = innerAngle > 0;
    const innerWheel = isLeftTurn ? { x: leftX, angle: innerRad } : { x: rightX, angle: innerRad };
    const outerWheel = isLeftTurn ? { x: rightX, angle: outerRad } : { x: leftX, angle: outerRad };

    // Draw inner wheel (pink)
    ctx.strokeStyle = "#ff5cf4";
    ctx.lineWidth = 4;
    ctx.save();
    ctx.translate(innerWheel.x, frontY);
    ctx.rotate(innerWheel.angle);
    ctx.beginPath();
    ctx.moveTo(0, -20);
    ctx.lineTo(0, 20);
    ctx.stroke();
    ctx.restore();

    // Draw outer wheel (cyan)
    ctx.strokeStyle = "#7df9ff";
    ctx.lineWidth = 4;
    ctx.save();
    ctx.translate(outerWheel.x, frontY);
    ctx.rotate(outerWheel.angle);
    ctx.beginPath();
    ctx.moveTo(0, -20);
    ctx.lineTo(0, 20);
    ctx.stroke();
    ctx.restore();

    // Rear wheels
    ctx.strokeStyle = "#a8b3d1";
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(leftX, rearY - 20);
    ctx.lineTo(leftX, rearY + 20);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(rightX, rearY - 20);
    ctx.lineTo(rightX, rearY + 20);
    ctx.stroke();

    ctx.restore();

    // Draw lines to ICR from both front wheels
    if (Math.abs(innerRad) > 0.001 && Math.abs(icrX - centerX) < w * 2) {
      ctx.setLineDash([5, 5]);

      // Line from inner wheel
      ctx.strokeStyle = "rgba(255, 92, 244, 0.5)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(centerX + innerWheel.x, centerY + frontY);
      ctx.lineTo(icrX, icrY);
      ctx.stroke();

      // Line from outer wheel
      ctx.strokeStyle = "rgba(125, 249, 255, 0.5)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(centerX + outerWheel.x, centerY + frontY);
      ctx.lineTo(icrX, icrY);
      ctx.stroke();

      ctx.setLineDash([]);

      // ICR point
      ctx.fillStyle = "#ffd700";
      ctx.beginPath();
      ctx.arc(icrX, icrY, 10, 0, Math.PI * 2);
      ctx.fill();

      ctx.font = "bold 14px monospace";
      ctx.fillStyle = "#ffd700";
      ctx.fillText("ICR", icrX + 15, icrY);
    }

    // Display angles
    ctx.font = "13px monospace";
    ctx.fillStyle = "#ff5cf4";
    ctx.fillText(`Inner: ${innerAngle.toFixed(1)}°`, 20, 30);
    ctx.fillStyle = "#7df9ff";
    ctx.fillText(`Outer: ${outerAngle.toFixed(1)}°`, 20, 50);

    if (Math.abs(innerRad) > 0.001) {
      ctx.fillStyle = "#a8b3d1";
      ctx.fillText(`Δ = ${Math.abs(innerAngle - outerAngle).toFixed(2)}°`, 20, 70);
    }

  }, [wheelbase, trackWidth, innerAngle]);

  return (
    <canvas
      ref={canvasRef}
      width={700}
      height={500}
      style={{
        width: "100%",
        height: "auto",
        border: "1px solid rgba(255,255,255,0.2)",
        borderRadius: "10px",
        background: "#0a0e1a"
      }}
    />
  );
}

export default function AckermannSteering() {
  const [innerAngle, setInnerAngle] = useState(0);
  const [wheelbase, setWheelbase] = useState(2.7);
  const [trackWidth, setTrackWidth] = useState(1.5);

  // Calculate outer angle using Ackermann formula
  const innerRad = (innerAngle * Math.PI) / 180;
  const cotInner = innerRad !== 0 ? 1 / Math.tan(innerRad) : Infinity;
  const cotOuter = cotInner + trackWidth / wheelbase;
  const outerAngle = cotOuter !== Infinity ? (Math.atan(1 / cotOuter) * 180) / Math.PI : 0;

  return (
    <div className="slide">
      <h2>Ackermann Steering Geometry</h2>

      <div className="slide-card">
        <div className="slide-card__title">Why Ackermann?</div>
        <p>
          In a turn, the <b>inner wheel</b> travels a tighter radius than the <b>outer wheel</b>.
          If both wheels had the same steering angle, they would fight each other (tire scrub).
          <b> Ackermann steering geometry</b> ensures both front wheels point toward the same
          instantaneous center of rotation (ICR), minimizing tire wear and improving handling.
        </p>
      </div>

      <div className="slide-columns">
        <div>
          <div className="slide-card">
            <div className="slide-card__title">Ackermann Formula</div>
            <div className="slide-code">
              cot(δₒ) = cot(δᵢ) + t/L
              <br /><br />
              Or equivalently:
              <br />
              tan(δₒ) = L / (R + t/2)
              <br />
              tan(δᵢ) = L / (R - t/2)
              <br /><br />
              Where:
              <br />• δₒ = Outer wheel angle
              <br />• δᵢ = Inner wheel angle
              <br />• L = Wheelbase
              <br />• t = Track width
              <br />• R = Turning radius
            </div>
          </div>

          <div className="slide-card" style={{ marginTop: ".75rem" }}>
            <div className="slide-card__title">Interactive Controls</div>
            <div style={{ display: "grid", gap: ".5rem" }}>
              <label style={{ display: "grid", gap: ".25rem" }}>
                <span>Inner Wheel Angle: {innerAngle}°</span>
                <input
                  type="range"
                  min="-30"
                  max="30"
                  step="0.5"
                  value={innerAngle}
                  onChange={(e) => setInnerAngle(Number(e.target.value))}
                  style={{ width: "100%" }}
                />
              </label>

              <div className="slide-code" style={{ fontSize: "0.9rem", padding: ".4rem" }}>
                Calculated Outer Angle: <b>{outerAngle.toFixed(2)}°</b>
                <br />
                Difference: <b>{Math.abs(innerAngle - outerAngle).toFixed(2)}°</b>
              </div>

              <label style={{ display: "grid", gap: ".25rem" }}>
                <span>Wheelbase: {wheelbase} m</span>
                <input
                  type="range"
                  min="2.0"
                  max="4.0"
                  step="0.1"
                  value={wheelbase}
                  onChange={(e) => setWheelbase(Number(e.target.value))}
                  style={{ width: "100%" }}
                />
              </label>

              <label style={{ display: "grid", gap: ".25rem" }}>
                <span>Track Width: {trackWidth} m</span>
                <input
                  type="range"
                  min="1.2"
                  max="2.0"
                  step="0.1"
                  value={trackWidth}
                  onChange={(e) => setTrackWidth(Number(e.target.value))}
                  style={{ width: "100%" }}
                />
              </label>
            </div>
          </div>
        </div>

        <div className="slide-figure">
          <AckermannVisualization
            wheelbase={wheelbase}
            trackWidth={trackWidth}
            innerAngle={innerAngle}
          />
          <figcaption>
            Top view: The <span style={{ color: "#ff5cf4" }}>inner wheel (pink)</span> has a
            larger steering angle than the <span style={{ color: "#7df9ff" }}>outer wheel (cyan)</span>.
            Both wheel axes intersect at the ICR (gold point).
          </figcaption>
        </div>
      </div>

      <div className="slide-callout slide-callout--info">
        <b>Real-World Application:</b> Most vehicles use an approximation of Ackermann geometry.
        Perfect Ackermann works best at low speeds. At higher speeds, tire slip angles become significant,
        and vehicles may use less than 100% Ackermann (or even anti-Ackermann for race cars).
      </div>
    </div>
  );
}
