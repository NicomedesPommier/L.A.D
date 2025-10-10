import React, { useState } from "react";
import { Handle, Position } from "@xyflow/react";

export default function ArgsNode({ id, data }) {
  const [raw, setRaw] = useState(
    Array.isArray(data.args) ? data.args.join(" ") : (data.default || "")
  );

  const update = (v) => {
    setRaw(v);
    const args = v.trim() ? v.trim().split(/\s+/) : [];
    data.onChange?.(id, { args });
  };

  const args = raw.trim() ? raw.trim().split(/\s+/) : [];

  return (
    <div className="rf-card" style={{ minWidth: 260 }}>
      <div className="rf-card__title">Args</div>
      <div className="rf-card__body">
        <input
          className="rf-input"
          value={raw}
          onChange={(e) => update(e.target.value)}
          placeholder="--ros-args -p background_r:=200"
        />
        <div className="rf-chips" style={{ marginTop: ".35rem" }}>
          {args.length
            ? args.map((a, i) => <span key={`${a}-${i}`} className="rf-chip">{a}</span>)
            : <span className="rf-chip rf-chip--ghost">Sin args</span>}
        </div>
      </div>
      <Handle type="source" position={Position.Right} id="out" />
    </div>
  );
}
