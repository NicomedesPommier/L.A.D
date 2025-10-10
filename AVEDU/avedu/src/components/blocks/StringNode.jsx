import React, { useState } from "react";
import { Handle, Position } from "@xyflow/react";

export default function StringNode({ id, data }) {
  const [value, setValue] = useState(String(data.value ?? ""));

  const change = (v) => {
    setValue(v);
    // Notificar solo cuando el usuario cambia, evita efecto infinito
    data.onChange?.(id, { value: v });
  };

  return (
    <div className="rf-card" style={{ minWidth: 220 }}>
      <div className="rf-card__title">{data.label || "Text"}</div>
      <div className="rf-card__body">
        <input
          className="rf-input"
          value={value}
          onChange={(e) => change(e.target.value)}
          placeholder={data.placeholder || "escribe aquí..."}
        />
      </div>
      <Handle type="source" position={Position.Right} id="out" />
    </div>
  );
}
