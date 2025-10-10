// components/blocks/UrdfRobotNode.jsx
import React from "react";
import { Handle, Position } from "@xyflow/react";

/**
 * URDF Robot Node
 * Representa un robot URDF que agrega links y joints entrantes.
 * Genera XML de robot y lo expone por el puerto 'xml'.
 *
 * data esperada:
 * { name?: string, xml?: string, onChange?: (id, patch) => void }
 */
export default function UrdfRobotNode({ id, data }) {
  const handleChange = (key, value) => {
    data?.onChange?.(id, { [key]: value });
  };

  return (
    <div className="rf-card" style={{ minWidth: 360 }}>
      <div className="rf-card__title">URDF Robot</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        <label className="rf-label">
          Nombre:
          <input
            type="text"
            className="rf-input"
            value={data?.name || ""}
            onChange={(e) => handleChange("name", e.target.value)}
          />
        </label>

        <details>
          <summary className="rf-label">XML generado</summary>
          <pre
            className="rf-terminal__code"
            style={{ maxHeight: 200, overflow: "auto", margin: 0 }}
          >
{data?.xml || "(vacío)"}
          </pre>
        </details>
      </div>

      {/* entran links y joints */}
      <Handle type="target" position={Position.Left} id="links" style={{ top: "30%" }} />
      <Handle type="target" position={Position.Left} id="joints" style={{ top: "70%" }} />

      {/* sale xml */}
      <Handle type="source" position={Position.Right} id="xml" />
    </div>
  );
}
