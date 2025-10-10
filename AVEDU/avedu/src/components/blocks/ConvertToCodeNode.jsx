import React from "react";
import { Handle, Position } from "@xyflow/react";

export default function ConvertToCodeNode({ data }) {
  const count = Number(data?.inCount || 0);
  const preview = data?.preview || "";

  return (
    <div className="rf-card" style={{ minWidth: 300 }}>
      <div className="rf-card__title">Convert2Code</div>
      <div className="rf-card__body" style={{ display: "grid", gap: 8 }}>
        <div className="rf-chip">{count} bloque(s) conectado(s)</div>
        <div style={{ opacity:.8, fontSize:12 }}>
          Conecta aquí <b>CreatePackage</b>, <b>RosRun</b>, etc. Lo que llegue se
          convertirá a comando.
        </div>
        <pre className="rfp-terminal__code" style={{ marginTop: 6, maxHeight: 140, overflow: "auto" }}>
{preview || "# (aún no hay nada conectado…)"}
        </pre>
      </div>

      <Handle type="target" position={Position.Left} id="in" />
      <Handle type="source" position={Position.Right} id="out" />
    </div>
  );
}
