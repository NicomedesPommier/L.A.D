// components/blocks/CenterMassNode.jsx
import React from "react";
import { Position } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";

/**
 * Center Mass Node - Extracts center point from geometry
 * Takes geometry and outputs the center as x,y,z and rotation
 * Useful for positioning TF frames at the center of URDF links
 */
export default function CenterMassNode({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const geometry = d.geometry || {};
  const center = d.center || { x: 0, y: 0, z: 0 };
  const rotation = d.rotation || { roll: 0, pitch: 0, yaw: 0 };

  return (
    <div className="rf-card rf-card--center-mass" style={{ minWidth: 320 }}>
      <div className="rf-card__title">⚖️ Center of Mass</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        {/* Geometry Info */}
        <div className="rf-field">
          <label>Geometry Source</label>
          <input
            className="rf-input"
            placeholder="Connected geometry"
            value={d.geometryName || ""}
            disabled
            style={{ opacity: 0.6 }}
          />
        </div>

        {/* Calculated Center */}
        <div style={{
          padding: ".6rem",
          background: "rgba(255, 152, 0, 0.1)",
          borderRadius: "6px",
          border: "1px solid rgba(255, 152, 0, 0.3)"
        }}>
          <div style={{ fontWeight: "bold", marginBottom: "0.4rem", fontSize: "0.85em" }}>
            📍 Calculated Center
          </div>
          <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: ".4rem", fontSize: "0.85em" }}>
            <div>
              <div style={{ opacity: 0.7, fontSize: "0.8em" }}>X</div>
              <div style={{ fontWeight: "bold", color: "#ff9800" }}>{center.x.toFixed(3)}m</div>
            </div>
            <div>
              <div style={{ opacity: 0.7, fontSize: "0.8em" }}>Y</div>
              <div style={{ fontWeight: "bold", color: "#ff9800" }}>{center.y.toFixed(3)}m</div>
            </div>
            <div>
              <div style={{ opacity: 0.7, fontSize: "0.8em" }}>Z</div>
              <div style={{ fontWeight: "bold", color: "#ff9800" }}>{center.z.toFixed(3)}m</div>
            </div>
          </div>
        </div>

        {/* Manual Offset (Optional) */}
        <div className="rf-field">
          <label>
            Offset (optional)
            <input
              type="checkbox"
              checked={d.useOffset || false}
              onChange={(e) => edit({ useOffset: e.target.checked })}
              style={{ marginLeft: "0.5rem" }}
            />
          </label>
          {d.useOffset && (
            <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: ".4rem", marginTop: "0.4rem" }}>
              <input
                className="rf-input"
                type="number"
                step="0.01"
                placeholder="+X"
                value={d.offset?.x || 0}
                onChange={(e) => edit({ offset: { ...(d.offset || {}), x: parseFloat(e.target.value) || 0 } })}
              />
              <input
                className="rf-input"
                type="number"
                step="0.01"
                placeholder="+Y"
                value={d.offset?.y || 0}
                onChange={(e) => edit({ offset: { ...(d.offset || {}), y: parseFloat(e.target.value) || 0 } })}
              />
              <input
                className="rf-input"
                type="number"
                step="0.01"
                placeholder="+Z"
                value={d.offset?.z || 0}
                onChange={(e) => edit({ offset: { ...(d.offset || {}), z: parseFloat(e.target.value) || 0 } })}
              />
            </div>
          )}
        </div>

        {/* Info */}
        <div style={{
          padding: ".4rem",
          background: "rgba(125, 249, 255, 0.1)",
          borderRadius: "4px",
          fontSize: "0.75em",
          opacity: 0.8
        }}>
          Automatically calculates the center point of connected geometry
        </div>
      </div>

      {/* Input handle */}
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="geometry"
        label="geometry"
        color="yellow"
      />

      {/* Output handles */}
      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="center_out"
        label="center"
        color="orange"
        top="40%"
      />
      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="rotation_out"
        label="rotation"
        color="orange"
        top="60%"
      />
    </div>
  );
}
