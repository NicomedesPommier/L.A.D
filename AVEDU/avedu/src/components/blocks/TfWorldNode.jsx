// components/blocks/TfWorldNode.jsx
import React from "react";
import { Position } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";

/**
 * TF World Node - Creates the root world frame for TF tree
 * This is the base frame that all other frames reference
 */
export default function TfWorldNode({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  return (
    <div className="rf-card rf-card--tf-world" style={{ minWidth: 280 }}>
      <div className="rf-card__title">🌍 TF World Frame</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        {/* Frame Name */}
        <div className="rf-field">
          <label>Frame ID</label>
          <input
            className="rf-input"
            placeholder="world"
            value={d.frameId || "world"}
            onChange={(e) => edit({ frameId: e.target.value })}
          />
          <div style={{ fontSize: "0.75em", opacity: 0.7, marginTop: "0.25rem" }}>
            Root frame for the TF tree (typically 'world' or 'map')
          </div>
        </div>

        {/* Info Box */}
        <div style={{
          padding: ".5rem",
          background: "rgba(125, 249, 255, 0.1)",
          borderRadius: "6px",
          border: "1px solid rgba(125, 249, 255, 0.3)",
          fontSize: "0.8em"
        }}>
          <div style={{ fontWeight: "bold", marginBottom: "0.25rem" }}>📌 World Frame</div>
          <div style={{ opacity: 0.9 }}>
            This is the root of your TF tree. All other frames will be positioned relative to this frame.
          </div>
        </div>
      </div>

      {/* Output handle */}
      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="world_frame"
        label="world"
        color="cyan"
      />
    </div>
  );
}
