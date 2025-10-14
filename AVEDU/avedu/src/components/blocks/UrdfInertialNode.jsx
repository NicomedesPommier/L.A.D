// components/blocks/UrdfInertialNode.jsx
import React from "react";
import { Handle, Position } from "@xyflow/react";

/**
 * URDF Inertial Node - Modular component for link inertia
 * Connects to UrdfLink node
 */
export default function UrdfInertialNode({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const inertia = d.inertia || {};
  const origin = d.origin || { xyz: [0, 0, 0], rpy: [0, 0, 0] };

  const setInertia = (key, value) => {
    edit({ inertia: { ...inertia, [key]: value } });
  };

  const setOriginXyz = (index, value) => {
    const xyz = [...(origin.xyz || [0, 0, 0])];
    xyz[index] = value;
    edit({ origin: { ...origin, xyz } });
  };

  const setOriginRpy = (index, value) => {
    const rpy = [...(origin.rpy || [0, 0, 0])];
    rpy[index] = value;
    edit({ origin: { ...origin, rpy } });
  };

  return (
    <div className="rf-card rf-card--inertial" style={{ minWidth: 320 }}>
      <div className="rf-card__title">⚖️ Inertial</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        {/* Mass */}
        <div className="rf-field">
          <label>Mass (kg)</label>
          <input
            className="rf-input"
            type="number"
            step="0.01"
            placeholder="1.0"
            value={d.mass ?? ""}
            onChange={(e) => edit({ mass: parseFloat(e.target.value) || 0 })}
          />
        </div>

        {/* Inertia tensor */}
        <div className="rf-field">
          <label>Inertia Tensor</label>
          <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr", gap: ".3rem" }}>
            <input
              className="rf-input"
              type="number"
              step="0.001"
              placeholder="ixx"
              title="ixx"
              value={inertia.ixx ?? ""}
              onChange={(e) => setInertia("ixx", parseFloat(e.target.value) || 0)}
            />
            <input
              className="rf-input"
              type="number"
              step="0.001"
              placeholder="iyy"
              title="iyy"
              value={inertia.iyy ?? ""}
              onChange={(e) => setInertia("iyy", parseFloat(e.target.value) || 0)}
            />
            <input
              className="rf-input"
              type="number"
              step="0.001"
              placeholder="izz"
              title="izz"
              value={inertia.izz ?? ""}
              onChange={(e) => setInertia("izz", parseFloat(e.target.value) || 0)}
            />
          </div>
          <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr", gap: ".3rem", marginTop: ".3rem" }}>
            <input
              className="rf-input"
              type="number"
              step="0.001"
              placeholder="ixy"
              title="ixy"
              value={inertia.ixy ?? 0}
              onChange={(e) => setInertia("ixy", parseFloat(e.target.value) || 0)}
            />
            <input
              className="rf-input"
              type="number"
              step="0.001"
              placeholder="ixz"
              title="ixz"
              value={inertia.ixz ?? 0}
              onChange={(e) => setInertia("ixz", parseFloat(e.target.value) || 0)}
            />
            <input
              className="rf-input"
              type="number"
              step="0.001"
              placeholder="iyz"
              title="iyz"
              value={inertia.iyz ?? 0}
              onChange={(e) => setInertia("iyz", parseFloat(e.target.value) || 0)}
            />
          </div>
        </div>

        {/* Origin */}
        <details>
          <summary className="rf-field__summary">Origin Transform</summary>
          <div className="rf-field">
            <label>Position (xyz)</label>
            <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr", gap: ".3rem" }}>
              {[0, 1, 2].map((i) => (
                <input
                  key={`xyz-${i}`}
                  className="rf-input"
                  type="number"
                  step="0.01"
                  placeholder={["x", "y", "z"][i]}
                  value={origin.xyz?.[i] ?? 0}
                  onChange={(e) => setOriginXyz(i, parseFloat(e.target.value) || 0)}
                />
              ))}
            </div>
          </div>

          <div className="rf-field">
            <label>Rotation (rpy)</label>
            <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr", gap: ".3rem" }}>
              {[0, 1, 2].map((i) => (
                <input
                  key={`rpy-${i}`}
                  className="rf-input"
                  type="number"
                  step="0.01"
                  placeholder={["r", "p", "y"][i]}
                  value={origin.rpy?.[i] ?? 0}
                  onChange={(e) => setOriginRpy(i, parseFloat(e.target.value) || 0)}
                />
              ))}
            </div>
          </div>
        </details>
      </div>

      {/* Output handle - connects to Link */}
      <Handle
        type="source"
        position={Position.Right}
        id="inertial"
        style={{
          width: "16px",
          height: "16px",
          background: "#ff9800",
          border: "3px solid #fff"
        }}
      />
    </div>
  );
}
