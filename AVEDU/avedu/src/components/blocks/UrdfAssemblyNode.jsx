// components/blocks/UrdfAssemblyNode.jsx
import React from "react";
import { Handle, Position } from "@xyflow/react";

/**
 * URDF Assembly Node - Groups links and joints before sending to Robot
 * Provides better organization for complex robots (Blender-style workflow)
 *
 * data esperada:
 * {
 *   id, name, description,
 *   links: [...],    // from UrdfLink nodes
 *   joints: [...],   // from UrdfJoint nodes
 *   onChange(id, patch)
 * }
 */
export default function UrdfAssemblyNode({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const links = Array.isArray(d.links) ? d.links : [];
  const joints = Array.isArray(d.joints) ? d.joints : [];

  return (
    <div className="rf-card rf-card--assembly" style={{ minWidth: 380 }}>
      <div className="rf-card__title">🔧 Assembly</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        {/* Assembly Name */}
        <div className="rf-field">
          <label>Assembly Name</label>
          <input
            className="rf-input"
            placeholder="arm_assembly"
            value={d.name || ""}
            onChange={(e) => edit({ name: e.target.value })}
          />
        </div>

        {/* Description */}
        <div className="rf-field">
          <label>Description (optional)</label>
          <input
            className="rf-input"
            placeholder="Left arm assembly with 3 joints"
            value={d.description || ""}
            onChange={(e) => edit({ description: e.target.value })}
          />
        </div>

        {/* Statistics */}
        <div className="rf-stats" style={{
          background: "#f5f5f5",
          padding: ".75rem",
          borderRadius: "4px",
          marginTop: ".5rem"
        }}>
          <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: ".5rem" }}>
            <div className="rf-stat-item">
              <div style={{ fontSize: "1.5em", fontWeight: "bold", color: "#4caf50" }}>
                {links.length}
              </div>
              <div style={{ fontSize: "0.9em", color: "#666" }}>Links</div>
            </div>
            <div className="rf-stat-item">
              <div style={{ fontSize: "1.5em", fontWeight: "bold", color: "#2196f3" }}>
                {joints.length}
              </div>
              <div style={{ fontSize: "0.9em", color: "#666" }}>Joints</div>
            </div>
          </div>
        </div>

        {/* Component list */}
        <details className="rf-details">
          <summary className="rf-field__summary">Components</summary>
          <div style={{ fontSize: "0.9em", maxHeight: "200px", overflowY: "auto" }}>
            {links.length > 0 && (
              <div style={{ marginBottom: ".5rem" }}>
                <strong>Links:</strong>
                <ul style={{ margin: ".25rem 0", paddingLeft: "1.5rem" }}>
                  {links.map((link, i) => (
                    <li key={`link-${i}`}>{link.name || `link_${i}`}</li>
                  ))}
                </ul>
              </div>
            )}
            {joints.length > 0 && (
              <div>
                <strong>Joints:</strong>
                <ul style={{ margin: ".25rem 0", paddingLeft: "1.5rem" }}>
                  {joints.map((joint, i) => (
                    <li key={`joint-${i}`}>
                      {joint.name || `joint_${i}`} ({joint.type})
                      {joint.parent && joint.child && ` - ${joint.parent} → ${joint.child}`}
                    </li>
                  ))}
                </ul>
              </div>
            )}
            {links.length === 0 && joints.length === 0 && (
              <div style={{ color: "#999", fontStyle: "italic", padding: ".5rem" }}>
                No components in this assembly yet
              </div>
            )}
          </div>
        </details>

        <div className="rf-hint" style={{ fontSize: "0.85em", color: "#666", marginTop: ".5rem" }}>
          💡 Group related links and joints together, then connect to the Robot node
        </div>
      </div>

      {/* Input handles - receive links and joints (LEFT side) */}
      <Handle
        type="target"
        position={Position.Left}
        id="links"
        style={{
          top: "35%",
          width: "18px",
          height: "18px",
          background: "#4caf50",
          border: "3px solid #fff"
        }}
        title="Connect Link nodes here"
      />
      <Handle
        type="target"
        position={Position.Left}
        id="joints"
        style={{
          top: "65%",
          width: "18px",
          height: "18px",
          background: "#2196f3",
          border: "3px solid #fff"
        }}
        title="Connect Joint nodes here"
      />

      {/* Output handle - sends assembly to Robot (RIGHT side) */}
      <Handle
        type="source"
        position={Position.Right}
        id="assembly"
        style={{
          width: "20px",
          height: "20px",
          background: "#9c27b0",
          border: "3px solid #fff"
        }}
        title="Connect to Robot node"
      />
    </div>
  );
}
