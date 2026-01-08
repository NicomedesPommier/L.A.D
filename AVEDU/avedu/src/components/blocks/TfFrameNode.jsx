// components/blocks/TfFrameNode.jsx
import React from "react";
import { Position } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";

/**
 * TF Frame Node - Creates a TF frame with position and rotation
 * Can be positioned manually or use center of mass from geometry
 */
export default function TfFrameNode({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const position = d.position || { x: 0, y: 0, z: 0 };
  const rotation = d.rotation || { roll: 0, pitch: 0, yaw: 0 };

  return (
    <div className="rf-card rf-card--tf-frame" style={{ minWidth: 320 }}>
      <div className="rf-card__title">📐 TF Frame</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        {/* Frame Name */}
        <div className="rf-field">
          <label>Frame ID</label>
          <input
            className="rf-input"
            placeholder="base_link"
            value={d.frameId || ""}
            onChange={(e) => edit({ frameId: e.target.value })}
          />
        </div>

        {/* Position */}
        <div className="rf-field">
          <label>Position (m)</label>
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

        {/* Rotation */}
        <div className="rf-field">
          <label>Rotation (rad)</label>
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
        top="70%"
      />

      {/* Output handle */}
      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="frame_out"
        label="frame"
        color="cyan"
      />
    </div>
  );
}
