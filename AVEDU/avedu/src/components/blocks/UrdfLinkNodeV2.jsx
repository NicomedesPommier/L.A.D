// components/blocks/UrdfLinkNodeV2.jsx
import React from "react";
import { Handle, Position } from "@xyflow/react";

/**
 * URDF Link Node V2 - Improved modular version
 * Accepts connections from Inertial, Visual, and Collision nodes (Blender-style)
 *
 * data esperada:
 * {
 *   id, name,
 *   inertial?: {...},  // from UrdfInertialNode
 *   visuals: [...],    // from UrdfVisualNode(s)
 *   collisions: [...], // from UrdfCollisionNode(s)
 *   onChange(id, patch)
 * }
 */
export default function UrdfLinkNodeV2({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const visuals = Array.isArray(d.visuals) ? d.visuals : [];
  const collisions = Array.isArray(d.collisions) ? d.collisions : [];
  const hasInertial = !!d.inertial;

  return (
    <div className="rf-card rf-card--link" style={{ minWidth: 360 }}>
      <div className="rf-card__title">🔗 URDF Link</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        {/* Link Name */}
        <div className="rf-field">
          <label>Link Name</label>
          <input
            className="rf-input"
            placeholder="base_link"
            value={d.name || ""}
            onChange={(e) => edit({ name: e.target.value })}
          />
        </div>

        {/* Status indicators */}
        <div className="rf-status-grid" style={{
          display: "grid",
          gridTemplateColumns: "1fr 1fr 1fr",
          gap: ".5rem",
          marginTop: ".5rem"
        }}>
          <div className={`rf-status-badge ${hasInertial ? 'active' : ''}`}>
            <span>⚖️</span>
            <span>Inertial</span>
            {hasInertial && <span className="rf-status-indicator">●</span>}
          </div>
          <div className={`rf-status-badge ${visuals.length > 0 ? 'active' : ''}`}>
            <span>👁️</span>
            <span>Visual ({visuals.length})</span>
            {visuals.length > 0 && <span className="rf-status-indicator">●</span>}
          </div>
          <div className={`rf-status-badge ${collisions.length > 0 ? 'active' : ''}`}>
            <span>🛡️</span>
            <span>Collision ({collisions.length})</span>
            {collisions.length > 0 && <span className="rf-status-indicator">●</span>}
          </div>
        </div>

        {/* Component summary */}
        <details className="rf-details">
          <summary className="rf-field__summary">Component Details</summary>
          <div style={{ fontSize: "0.9em", padding: ".5rem", background: "#f5f5f5", borderRadius: "4px" }}>
            {hasInertial && (
              <div>
                <strong>Inertial:</strong> mass={d.inertial?.mass?.toFixed(2) || "0.00"} kg
              </div>
            )}
            {visuals.map((v, i) => (
              <div key={`v-${i}`}>
                <strong>Visual {i + 1}:</strong> {v.geometry?.type || "unknown"}
                {v.geometry?.filename && ` (${v.geometry.filename.split('/').pop()})`}
              </div>
            ))}
            {collisions.map((c, i) => (
              <div key={`c-${i}`}>
                <strong>Collision {i + 1}:</strong> {c.geometry?.type || "unknown"}
              </div>
            ))}
            {!hasInertial && visuals.length === 0 && collisions.length === 0 && (
              <div style={{ color: "#999", fontStyle: "italic" }}>
                No components connected. Connect Inertial, Visual, and Collision nodes to this link.
              </div>
            )}
          </div>
        </details>

        <div className="rf-hint" style={{ fontSize: "0.85em", color: "#666", marginTop: ".5rem" }}>
          💡 Connect Inertial, Visual, and Collision nodes to the left handles
        </div>
      </div>

      {/* Input handles - receive from component nodes (LEFT side) */}
      <Handle
        type="target"
        position={Position.Left}
        id="inertial"
        style={{
          top: "25%",
          width: "16px",
          height: "16px",
          background: "#ff9800",
          border: "3px solid #fff"
        }}
        title="Connect Inertial node here"
      />
      <Handle
        type="target"
        position={Position.Left}
        id="visual"
        style={{
          top: "50%",
          width: "16px",
          height: "16px",
          background: "#2196f3",
          border: "3px solid #fff"
        }}
        title="Connect Visual nodes here (multiple supported)"
      />
      <Handle
        type="target"
        position={Position.Left}
        id="collision"
        style={{
          top: "75%",
          width: "16px",
          height: "16px",
          background: "#f44336",
          border: "3px solid #fff"
        }}
        title="Connect Collision nodes here (multiple supported)"
      />

      {/* Output handle - sends to Robot/Assembly (RIGHT side) */}
      <Handle
        type="source"
        position={Position.Right}
        id="link"
        style={{
          width: "20px",
          height: "20px",
          background: "#4caf50",
          border: "3px solid #fff"
        }}
        title="Connect to Assembly or Robot node"
      />
    </div>
  );
}
