// components/blocks/TfChildNode.jsx
import React from "react";
import { Position } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";

/**
 * TF Child Node - Connects parent and child TF frames
 * Represents a parent-child relationship in the TF tree
 */
export default function TfChildNode({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const position = d.position || { x: 0, y: 0, z: 0 };
  const rotation = d.rotation || { roll: 0, pitch: 0, yaw: 0 };

  return (
    <div className="rf-card rf-card--tf-child" style={{ minWidth: 320 }}>
      <div className="rf-card__title">🔗 TF Child Frame</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        {/* Child Frame Name */}
        <div className="rf-field">
          <label>Child Frame ID</label>
          <input
            className="rf-input"
            placeholder="child_frame"
            value={d.childFrameId || ""}
            onChange={(e) => edit({ childFrameId: e.target.value })}
          />
        </div>

        {/* Relative Position */}
        <div className="rf-field">
          <label>Relative Position (m)</label>
          <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: ".4rem" }}>
            <input
              className="rf-input"
              type="number"
              step="0.1"
              placeholder="X"
              value={position.x}
              onChange={(e) => edit({ position: { ...position, x: parseFloat(e.target.value) || 0 } })}
            />
            <input
              className="rf-input"
              type="number"
              step="0.1"
              placeholder="Y"
              value={position.y}
              onChange={(e) => edit({ position: { ...position, y: parseFloat(e.target.value) || 0 } })}
            />
            <input
              className="rf-input"
              type="number"
              step="0.1"
              placeholder="Z"
              value={position.z}
              onChange={(e) => edit({ position: { ...position, z: parseFloat(e.target.value) || 0 } })}
            />
          </div>
        </div>

        {/* Relative Rotation */}
        <div className="rf-field">
          <label>Relative Rotation (rad)</label>
          <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: ".4rem" }}>
            <input
              className="rf-input"
              type="number"
              step="0.1"
              placeholder="Roll"
              value={rotation.roll}
              onChange={(e) => edit({ rotation: { ...rotation, roll: parseFloat(e.target.value) || 0 } })}
            />
            <input
              className="rf-input"
              type="number"
              step="0.1"
              placeholder="Pitch"
              value={rotation.pitch}
              onChange={(e) => edit({ rotation: { ...rotation, pitch: parseFloat(e.target.value) || 0 } })}
            />
            <input
              className="rf-input"
              type="number"
              step="0.1"
              placeholder="Yaw"
              value={rotation.yaw}
              onChange={(e) => edit({ rotation: { ...rotation, yaw: parseFloat(e.target.value) || 0 } })}
            />
          </div>
        </div>

        {/* Broadcast Type */}
        <div className="rf-field">
          <label>Broadcast Type</label>
          <select
            className="rf-input"
            value={d.broadcastType || "static"}
            onChange={(e) => edit({ broadcastType: e.target.value })}
          >
            <option value="static">Static (tf_static)</option>
            <option value="dynamic">Dynamic (tf)</option>
          </select>
        </div>

        {/* Info */}
        <div style={{
          padding: ".4rem",
          background: "rgba(33, 150, 243, 0.1)",
          borderRadius: "4px",
          fontSize: "0.75em",
          opacity: 0.8
        }}>
          Position and rotation are relative to parent frame
        </div>
      </div>

      {/* Input handles */}
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="parent_frame"
        label="parent"
        color="cyan"
        top="30%"
      />
      <HandleWithLabel
        type="target"
        position={Position.Left}
        id="center_mass"
        label="center"
        color="orange"
        top="60%"
      />

      {/* Output handle */}
      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="child_frame_out"
        label="child"
        color="cyan"
      />
    </div>
  );
}
