// src/levels/slidesVehicleDynamics/02-InstantaneousCenterRotation.jsx
import React, { useState, useRef, useEffect } from "react";

export const meta = {
  id: "vd-icr",
  title: "Instantaneous Center of Rotation (ICR)",
  order: 2,
  objectiveCode: "vd-slide-icr",
};

function ICRVisualization({ wheelbase, trackWidth, steeringAngle }) {
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

    // Scale and center
    const scale = 80;
    const centerX = w / 2;
    const centerY = h / 2 + 50;

    // Convert steering angle to radians
    const deltaRad = (steeringAngle * Math.PI) / 180;

    // Calculate turning radius and ICR position
    let R = Infinity; // Turning radius
    let icrX = centerX;
    let icrY = centerY;

    if (Math.abs(deltaRad) > 0.001) {
      R = wheelbase / Math.tan(Math.abs(deltaRad));
      const sign = deltaRad > 0 ? -1 : 1; // Left turn is negative x
      icrX = centerX + sign * R * scale;
      icrY = centerY;
    }

    // Draw vehicle (simplified top view)
    ctx.save();
    ctx.translate(centerX, centerY);

    // Vehicle body (rectangle)
    ctx.strokeStyle = "#7df9ff";
    ctx.lineWidth = 3;
    ctx.strokeRect(
      -trackWidth * scale / 2,
      -wheelbase * scale / 2,
      trackWidth * scale,
      wheelbase * scale
    );

    // Front wheels
    const frontY = -wheelbase * scale / 2;
    const rearY = wheelbase * scale / 2;
    const leftX = -trackWidth * scale / 2;
    const rightX = trackWidth * scale / 2;

    // Draw front wheels with steering angle
    ctx.strokeStyle = "#ff5cf4";
    ctx.lineWidth = 4;

    // Left front wheel
    ctx.save();
    ctx.translate(leftX, frontY);
    ctx.rotate(deltaRad);
    ctx.beginPath();
    ctx.moveTo(-10, -15);
    ctx.lineTo(-10, 15);
    ctx.stroke();
    ctx.restore();

    // Right front wheel
    ctx.save();
    ctx.translate(rightX, frontY);
    ctx.rotate(deltaRad);
    ctx.beginPath();
    ctx.moveTo(10, -15);
    ctx.lineTo(10, 15);
    ctx.stroke();
    ctx.restore();

    // Rear wheels (no steering)
    ctx.strokeStyle = "#a8b3d1";
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(leftX - 10, rearY - 15);
    ctx.lineTo(leftX - 10, rearY + 15);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(rightX + 10, rearY - 15);
    ctx.lineTo(rightX + 10, rearY + 15);
    ctx.stroke();

    ctx.restore();

    // Draw ICR if turning
    if (Math.abs(deltaRad) > 0.001 && Math.abs(icrX - centerX) < w * 2) {
      // ICR point
      ctx.fillStyle = "#ff5cf4";
      ctx.beginPath();
      ctx.arc(icrX, icrY, 8, 0, Math.PI * 2);
      ctx.fill();

      // Label
      ctx.font = "14px monospace";
      ctx.fillStyle = "#ff5cf4";
      ctx.fillText("ICR", icrX + 15, icrY - 10);

      // Line from rear axle center to ICR
      ctx.strokeStyle = "rgba(255, 92, 244, 0.5)";
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(centerX, centerY + (wheelbase * scale / 2));
      ctx.lineTo(icrX, icrY);
      ctx.stroke();
      ctx.setLineDash([]);

      // Turning circle arc
      ctx.strokeStyle = "rgba(125, 249, 255, 0.3)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      const radius = Math.sqrt(
        Math.pow(icrX - centerX, 2) + Math.pow(icrY - centerY, 2)
      );
      ctx.arc(icrX, icrY, radius, 0, Math.PI * 2);
      ctx.stroke();

      // Display turning radius
      ctx.font = "12px monospace";
      ctx.fillStyle = "#7df9ff";
      ctx.fillText(`R = ${R.toFixed(2)} m`, 20, h - 20);
    } else {
      ctx.font = "14px monospace";
      ctx.fillStyle = "#a8b3d1";
      ctx.fillText("Straight ahead (ICR at infinity)", centerX - 120, h - 20);
    }

    // Axis labels
    ctx.font = "12px monospace";
    ctx.fillStyle = "#a8b3d1";
    ctx.fillText("Front", centerX - 20, centerY - wheelbase * scale / 2 - 15);
    ctx.fillText("Rear", centerX - 15, centerY + wheelbase * scale / 2 + 30);

  }, [wheelbase, trackWidth, steeringAngle]);

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

export default function InstantaneousCenterRotation() {
  const [steeringAngle, setSteeringAngle] = useState(0);
  const [wheelbase, setWheelbase] = useState(2.7);
  const [trackWidth, setTrackWidth] = useState(1.5);

  return (
    <div className="slide">
      <h2>Instantaneous Center of Rotation (ICR)</h2>

      <div className="slide-card">
        <div className="slide-card__title">Concept</div>
        <p>
          The <b>Instantaneous Center of Rotation (ICR)</b> is the point about which a vehicle
          appears to rotate at any given instant. For a vehicle with front-wheel steering,
          the ICR lies on the extension of the rear axle line.
        </p>
      </div>

      <div className="slide-columns">
        <div>
          <div className="slide-card">
            <div className="slide-card__title">Key Formula</div>
            <div className="slide-code">
              R = L / tan(δ)
              <br /><br />
              Where:
              <br />• R = Turning radius
              <br />• L = Wheelbase
              <br />• δ = Steering angle
            </div>
          </div>

          <div className="slide-card" style={{ marginTop: ".75rem" }}>
            <div className="slide-card__title">Interactive Controls</div>
            <div style={{ display: "grid", gap: ".5rem" }}>
              <label style={{ display: "grid", gap: ".25rem" }}>
                <span>Steering Angle: {steeringAngle}°</span>
                <input
                  type="range"
                  min="-30"
                  max="30"
                  step="1"
                  value={steeringAngle}
                  onChange={(e) => setSteeringAngle(Number(e.target.value))}
                  style={{ width: "100%" }}
                />
              </label>

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
          <ICRVisualization
            wheelbase={wheelbase}
            trackWidth={trackWidth}
            steeringAngle={steeringAngle}
          />
          <figcaption>
            Top view: Adjust the steering angle to see how the ICR position changes.
            The pink dot shows the ICR location, and the dashed line shows the turning radius.
          </figcaption>
        </div>
      </div>

      <div className="slide-callout slide-callout--info">
        <b>Key Insight:</b> When the steering angle is zero (straight ahead), the ICR is at infinity.
        As you increase the steering angle, the turning radius decreases and the ICR moves closer to the vehicle.
      </div>
    </div>
  );
}
