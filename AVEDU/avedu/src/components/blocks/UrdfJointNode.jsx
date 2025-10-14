// components/blocks/UrdfJointNode.jsx
import React from "react";
import { Handle, Position } from "@xyflow/react";

/**
 * URDF Joint
 * data esperada:
 * {
 *   id, name, type: 'fixed'|'revolute'|'continuous'|'prismatic'|'floating'|'planar',
 *   parent, child,
 *   origin: { xyz:[x,y,z], rpy:[r,p,y] },
 *   axis: { xyz:[x,y,z] },
 *   onChange(id, patch)
 * }
 */
export default function UrdfJointNode({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const setOrigin = (k, val) => {
    const base = d.origin || {};
    const arr = [...(base.xyz || [0, 0, 0])];
    arr[k] = val;
    edit({ origin: { ...base, xyz: arr } });
  };

  const setRpy = (k, val) => {
    const base = d.origin || {};
    const arr = [...(base.rpy || [0, 0, 0])];
    arr[k] = val;
    edit({ origin: { ...base, rpy: arr } });
  };

  const setAxis = (k, val) => {
    const base = d.axis || {};
    const arr = [...(base.xyz || [1, 0, 0])];
    arr[k] = val;
    edit({ axis: { xyz: arr } });
  };

  return (
    <div className="rf-card" style={{ minWidth: 340 }}>
      <div className="rf-card__title">URDF Joint</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        <div className="rf-field">
          <span>Name</span>
          <input
            className="rf-input"
            value={d.name || ""}
            placeholder="joint1"
            onChange={(e) => edit({ name: e.target.value })}
          />
        </div>

        <div className="rf-inline">
          <span>Type</span>
          <select
            className="rf-input"
            value={d.type || "fixed"}
            onChange={(e) => edit({ type: e.target.value })}
          >
            <option value="fixed">fixed</option>
            <option value="revolute">revolute</option>
            <option value="continuous">continuous</option>
            <option value="prismatic">prismatic</option>
            <option value="floating">floating</option>
            <option value="planar">planar</option>
          </select>
        </div>

        <div className="rf-inline">
          <span>Parent link</span>
          <input
            className="rf-input"
            value={d.parent || ""}
            placeholder="base_link"
            onChange={(e) => edit({ parent: e.target.value })}
          />
        </div>

        <div className="rf-inline">
          <span>Child link</span>
          <input
            className="rf-input"
            value={d.child || ""}
            placeholder="link1"
            onChange={(e) => edit({ child: e.target.value })}
          />
        </div>

        <div className="rf-inline">
          <span>Origin xyz</span>
          {[0, 1, 2].map((k) => (
            <input
              key={`xyz-${k}`}
              className="rf-input"
              type="number"
              step="any"
              value={d.origin?.xyz?.[k] ?? 0}
              onChange={(e) => setOrigin(k, Number(e.target.value))}
            />
          ))}
        </div>

        <div className="rf-inline">
          <span>Origin rpy</span>
          {[0, 1, 2].map((k) => (
            <input
              key={`rpy-${k}`}
              className="rf-input"
              type="number"
              step="any"
              value={d.origin?.rpy?.[k] ?? 0}
              onChange={(e) => setRpy(k, Number(e.target.value))}
            />
          ))}
        </div>

        <div className="rf-inline">
          <span>Axis xyz</span>
          {[0, 1, 2].map((k) => (
            <input
              key={`axis-${k}`}
              className="rf-input"
              type="number"
              step="any"
              value={d.axis?.xyz?.[k] ?? (k === 0 ? 1 : 0)}
              onChange={(e) => setAxis(k, Number(e.target.value))}
            />
          ))}
        </div>
      </div>

      {/* salida: emite el objeto joint por 'out' */}
      <Handle
        type="source"
        position={Position.Right}
        id="out"
        style={{
          width: "18px",
          height: "18px",
          background: "#2196f3",
          border: "3px solid #fff"
        }}
        title="Connect to Assembly or Robot node"
      />
    </div>
  );
}
