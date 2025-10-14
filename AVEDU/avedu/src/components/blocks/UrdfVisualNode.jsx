// components/blocks/UrdfVisualNode.jsx
import React from "react";
import { Handle, Position } from "@xyflow/react";

/**
 * URDF Visual Node - Modular component for link visual geometry
 * Connects to UrdfLink node
 */
export default function UrdfVisualNode({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const geometry = d.geometry || { type: "box", size: [1, 1, 1] };
  const origin = d.origin || { xyz: [0, 0, 0], rpy: [0, 0, 0] };
  const material = d.material || {};

  const setGeometry = (patch) => {
    edit({ geometry: { ...geometry, ...patch } });
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

  const setSize = (index, value) => {
    const size = [...(geometry.size || [1, 1, 1])];
    size[index] = value;
    setGeometry({ size });
  };

  return (
    <div className="rf-card rf-card--visual" style={{ minWidth: 320 }}>
      <div className="rf-card__title">👁️ Visual Geometry</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        {/* Geometry Type */}
        <div className="rf-field">
          <label>Geometry Type</label>
          <select
            className="rf-input"
            value={geometry.type || "box"}
            onChange={(e) => {
              const type = e.target.value;
              const newGeom = { type };

              // Set defaults based on type
              if (type === "mesh") {
                newGeom.filename = geometry.filename || "";
                newGeom.scale = geometry.scale || [1, 1, 1];
              } else if (type === "box") {
                newGeom.size = geometry.size || [1, 1, 1];
              } else if (type === "cylinder") {
                newGeom.radius = geometry.radius || 0.5;
                newGeom.length = geometry.length || 1;
              } else if (type === "sphere") {
                newGeom.radius = geometry.radius || 0.5;
              }

              edit({ geometry: newGeom });
            }}
          >
            <option value="mesh">Mesh</option>
            <option value="box">Box</option>
            <option value="cylinder">Cylinder</option>
            <option value="sphere">Sphere</option>
          </select>
        </div>

        {/* Type-specific fields */}
        {geometry.type === "mesh" && (
          <>
            <div className="rf-field">
              <label>Mesh File</label>
              <input
                className="rf-input"
                placeholder="package://path/to/mesh.dae"
                value={geometry.filename || ""}
                onChange={(e) => setGeometry({ filename: e.target.value })}
              />
            </div>
            <div className="rf-field">
              <label>Scale</label>
              <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr", gap: ".3rem" }}>
                {[0, 1, 2].map((i) => (
                  <input
                    key={`scale-${i}`}
                    className="rf-input"
                    type="number"
                    step="0.1"
                    placeholder={["x", "y", "z"][i]}
                    value={geometry.scale?.[i] ?? 1}
                    onChange={(e) => {
                      const scale = [...(geometry.scale || [1, 1, 1])];
                      scale[i] = parseFloat(e.target.value) || 1;
                      setGeometry({ scale });
                    }}
                  />
                ))}
              </div>
            </div>
          </>
        )}

        {geometry.type === "box" && (
          <div className="rf-field">
            <label>Size (x y z)</label>
            <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr", gap: ".3rem" }}>
              {[0, 1, 2].map((i) => (
                <input
                  key={`size-${i}`}
                  className="rf-input"
                  type="number"
                  step="0.1"
                  placeholder={["width", "depth", "height"][i]}
                  value={geometry.size?.[i] ?? 1}
                  onChange={(e) => setSize(i, parseFloat(e.target.value) || 1)}
                />
              ))}
            </div>
          </div>
        )}

        {(geometry.type === "cylinder" || geometry.type === "sphere") && (
          <div className="rf-field">
            <label>Radius</label>
            <input
              className="rf-input"
              type="number"
              step="0.1"
              placeholder="0.5"
              value={geometry.radius ?? 0.5}
              onChange={(e) => setGeometry({ radius: parseFloat(e.target.value) || 0.5 })}
            />
          </div>
        )}

        {geometry.type === "cylinder" && (
          <div className="rf-field">
            <label>Length</label>
            <input
              className="rf-input"
              type="number"
              step="0.1"
              placeholder="1.0"
              value={geometry.length ?? 1}
              onChange={(e) => setGeometry({ length: parseFloat(e.target.value) || 1 })}
            />
          </div>
        )}

        {/* Material */}
        <details>
          <summary className="rf-field__summary">Material</summary>
          <div className="rf-field">
            <label>Material Name</label>
            <input
              className="rf-input"
              placeholder="blue_material"
              value={material.name || ""}
              onChange={(e) => edit({ material: { ...material, name: e.target.value } })}
            />
          </div>
          <div className="rf-field">
            <label>Color (RGBA)</label>
            <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr 1fr", gap: ".3rem" }}>
              {["r", "g", "b", "a"].map((c, i) => (
                <input
                  key={c}
                  className="rf-input"
                  type="number"
                  step="0.1"
                  min="0"
                  max="1"
                  placeholder={c}
                  value={material.color?.[i] ?? (c === "a" ? 1 : 0.5)}
                  onChange={(e) => {
                    const color = [...(material.color || [0.5, 0.5, 0.5, 1])];
                    color[i] = parseFloat(e.target.value) || 0;
                    edit({ material: { ...material, color } });
                  }}
                />
              ))}
            </div>
          </div>
        </details>

        {/* Origin Transform */}
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
        id="visual"
        style={{
          width: "16px",
          height: "16px",
          background: "#2196f3",
          border: "3px solid #fff"
        }}
      />
    </div>
  );
}
