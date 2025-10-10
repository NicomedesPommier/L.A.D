import React from "react";
import { Handle, Position } from "@xyflow/react";

/**
 * Nodo sumidero: recibe pkg/exe/ns/args por puertos.
 * Sólo muestra un resumen (no edita).
 * Puertos de entrada:
 *  - pkg   (string)
 *  - exe   (string)
 *  - ns    (string)
 *  - args  (string[])
 */
export default function RosRunNode({ data }) {
  const { pkg, exe, ns, args } = data;

  return (
    <div className="rf-card" style={{ minWidth: 320 }}>
      <div className="rf-card__title">ROS Run</div>
      <div className="rf-card__body" style={{ display: "grid", gap: ".4rem" }}>
        <div className="rf-field">
          <span>Package</span>
          <div className="rf-input" style={{ pointerEvents: "none" }}>{pkg || <i>(vacío)</i>}</div>
        </div>
        <div className="rf-field">
          <span>Executable</span>
          <div className="rf-input" style={{ pointerEvents: "none" }}>{exe || <i>(vacío)</i>}</div>
        </div>
        <div className="rf-field">
          <span>Namespace</span>
          <div className="rf-input" style={{ pointerEvents: "none" }}>{ns || <i>(sin ns)</i>}</div>
        </div>
        <div className="rf-field">
          <span>Args</span>
          <div className="rf-chips">
            {Array.isArray(args) && args.length
              ? args.map((a, i) => <span key={`${a}-${i}`} className="rf-chip">{a}</span>)
              : <span className="rf-chip rf-chip--ghost">Sin args</span>}
          </div>
        </div>
      </div>

      {/* entradas */}
      <Handle type="target" position={Position.Left} id="pkg" style={{ top: "30%" }} />
      <Handle type="target" position={Position.Left} id="exe" style={{ top: "50%" }} />
      <Handle type="target" position={Position.Left} id="ns"  style={{ top: "72%" }} />
      <Handle type="target" position={Position.Left} id="args" style={{ top: "90%" }} />

      {/* salida por si más adelante encadenamos */}
      <Handle type="source" position={Position.Right} id="out" />
    </div>
  );
}
