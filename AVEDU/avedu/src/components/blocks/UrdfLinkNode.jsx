// components/blocks/UrdfLinkNode.jsx
import React from "react";
import { Handle, Position } from "@xyflow/react";

/**
 * URDF Link
 * data esperada:
 * {
 *   id, name,
 *   visuals: [{ geometry:{type:'mesh'|'box'|'cylinder'|'sphere', filename?, size?, radius?, length?, scale?},
 *               origin:{ xyz:[x,y,z], rpy:[r,p,y] }, material? }],
 *   collisions: [{ geometry:{...}, origin:{...} }],
 *   inertial: { mass, inertia:{ ixx, iyy, izz, ixy?, ixz?, iyz? }, origin:{ xyz, rpy } },
 *   onChange(id, patch)
 * }
 */
export default function UrdfLinkNode({ id, data }) {
  const d = data || {};
  const visuals = Array.isArray(d.visuals) ? d.visuals : [];
  const collisions = Array.isArray(d.collisions) ? d.collisions : [];
  const inertial = d.inertial || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const setInertial = (patch) => edit({ inertial: { ...(d.inertial || {}), ...patch } });

  return (
    <div className="rf-card" style={{ minWidth: 360 }}>
      <div className="rf-card__title">URDF Link</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        {/* Nombre */}
        <div className="rf-field">
          <span>Name</span>
          <input
            className="rf-input"
            placeholder="base_link"
            value={d.name || ""}
            onChange={(e) => edit({ name: e.target.value })}
          />
        </div>

        {/* Inertial */}
        <details>
          <summary className="rf-field__summary">Inertial</summary>

          <div className="rf-field">
            <span>Mass</span>
            <input
              className="rf-input"
              type="number"
              step="any"
              placeholder="1.0"
              value={inertial.mass ?? ""}
              onChange={(e) => setInertial({ mass: Number(e.target.value) })}
            />
          </div>

          <div className="rf-inline">
            <span>Inertia (ixx, iyy, izz)</span>
            <input
              className="rf-input"
              type="number"
              step="any"
              placeholder="0.01"
              value={inertial.inertia?.ixx ?? ""}
              onChange={(e) =>
                setInertial({
                  inertia: { ...(inertial.inertia || {}), ixx: Number(e.target.value) },
                })
              }
            />
            <input
              className="rf-input"
              type="number"
              step="any"
              placeholder="0.01"
              value={inertial.inertia?.iyy ?? ""}
              onChange={(e) =>
                setInertial({
                  inertia: { ...(inertial.inertia || {}), iyy: Number(e.target.value) },
                })
              }
            />
            <input
              className="rf-input"
              type="number"
              step="any"
              placeholder="0.01"
              value={inertial.inertia?.izz ?? ""}
              onChange={(e) =>
                setInertial({
                  inertia: { ...(inertial.inertia || {}), izz: Number(e.target.value) },
                })
              }
            />
          </div>

          <div className="rf-inline">
            <span>Origin xyz</span>
            {[0, 1, 2].map((k) => (
              <input
                key={k}
                className="rf-input"
                type="number"
                step="any"
                value={inertial.origin?.xyz?.[k] ?? 0}
                onChange={(e) => {
                  const arr = [...(inertial.origin?.xyz || [0, 0, 0])];
                  arr[k] = Number(e.target.value);
                  setInertial({ origin: { ...(inertial.origin || {}), xyz: arr } });
                }}
              />
            ))}
          </div>

          <div className="rf-inline">
            <span>Origin rpy</span>
            {[0, 1, 2].map((k) => (
              <input
                key={k}
                className="rf-input"
                type="number"
                step="any"
                value={inertial.origin?.rpy?.[k] ?? 0}
                onChange={(e) => {
                  const arr = [...(inertial.origin?.rpy || [0, 0, 0])];
                  arr[k] = Number(e.target.value);
                  setInertial({ origin: { ...(inertial.origin || {}), rpy: arr } });
                }}
              />
            ))}
          </div>
        </details>

        {/* Visuals */}
        <details open>
          <summary className="rf-field__summary">Visuals</summary>
          {visuals.map((vv, i) => (
            <div key={i} className="rf-box" style={{ display: "grid", gap: ".4rem" }}>
              <div className="rf-inline">
                <select
                  className="rf-input"
                  value={vv.geometry?.type || "mesh"}
                  onChange={(e) => {
                    const type = e.target.value;
                    const next = [...visuals];
                    next[i] = {
                      ...(vv || {}),
                      geometry: {
                        type,
                        ...(type === "mesh" ? { filename: vv.geometry?.filename || "" } : {}),
                      },
                    };
                    edit({ visuals: next });
                  }}
                >
                  <option value="mesh">mesh</option>
                  <option value="box">box</option>
                  <option value="cylinder">cylinder</option>
                  <option value="sphere">sphere</option>
                </select>

                {vv.geometry?.type === "mesh" && (
                  <input
                    className="rf-input"
                    placeholder="package://path/to.dae | .stl"
                    value={vv.geometry?.filename || ""}
                    onChange={(e) => {
                      const next = [...visuals];
                      next[i] = {
                        ...(vv || {}),
                        geometry: { ...(vv.geometry || {}), type: "mesh", filename: e.target.value },
                      };
                      edit({ visuals: next });
                    }}
                  />
                )}
              </div>

              {/* Origin xyz/rpy */}
              <div className="rf-inline">
                <span>Origin xyz</span>
                {[0, 1, 2].map((k) => (
                  <input
                    key={k}
                    className="rf-input"
                    type="number"
                    step="any"
                    value={vv.origin?.xyz?.[k] ?? 0}
                    onChange={(e) => {
                      const next = [...visuals];
                      const arr = [...(vv.origin?.xyz || [0, 0, 0])];
                      arr[k] = Number(e.target.value);
                      next[i] = { ...(vv || {}), origin: { ...(vv.origin || {}), xyz: arr } };
                      edit({ visuals: next });
                    }}
                  />
                ))}
              </div>
              <div className="rf-inline">
                <span>Origin rpy</span>
                {[0, 1, 2].map((k) => (
                  <input
                    key={k}
                    className="rf-input"
                    type="number"
                    step="any"
                    value={vv.origin?.rpy?.[k] ?? 0}
                    onChange={(e) => {
                      const next = [...visuals];
                      const arr = [...(vv.origin?.rpy || [0, 0, 0])];
                      arr[k] = Number(e.target.value);
                      next[i] = { ...(vv || {}), origin: { ...(vv.origin || {}), rpy: arr } };
                      edit({ visuals: next });
                    }}
                  />
                ))}
              </div>

              <button
                className="btn danger"
                onClick={() => {
                  const next = visuals.filter((_, j) => j !== i);
                  edit({ visuals: next });
                }}
              >
                Eliminar visual
              </button>
            </div>
          ))}

          <button
            className="btn"
            onClick={() =>
              edit({
                visuals: [
                  ...visuals,
                  { geometry: { type: "mesh", filename: "" }, origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] } },
                ],
              })
            }
          >
            + Visual
          </button>
        </details>

        {/* Collisions */}
        <details>
          <summary className="rf-field__summary">Collision</summary>
          {collisions.map((cc, i) => (
            <div key={i} className="rf-box" style={{ display: "grid", gap: ".4rem" }}>
              <div className="rf-inline">
                <select
                  className="rf-input"
                  value={cc.geometry?.type || "mesh"}
                  onChange={(e) => {
                    const type = e.target.value;
                    const next = [...collisions];
                    next[i] = {
                      ...(cc || {}),
                      geometry: {
                        type,
                        ...(type === "mesh" ? { filename: cc.geometry?.filename || "" } : {}),
                      },
                    };
                    edit({ collisions: next });
                  }}
                >
                  <option value="mesh">mesh</option>
                  <option value="box">box</option>
                  <option value="cylinder">cylinder</option>
                  <option value="sphere">sphere</option>
                </select>

                {cc.geometry?.type === "mesh" && (
                  <input
                    className="rf-input"
                    placeholder="package://path/collision.dae"
                    value={cc.geometry?.filename || ""}
                    onChange={(e) => {
                      const next = [...collisions];
                      next[i] = {
                        ...(cc || {}),
                        geometry: { ...(cc.geometry || {}), type: "mesh", filename: e.target.value },
                      };
                      edit({ collisions: next });
                    }}
                  />
                )}
              </div>

              <div className="rf-inline">
                <span>Origin xyz</span>
                {[0, 1, 2].map((k) => (
                  <input
                    key={k}
                    className="rf-input"
                    type="number"
                    step="any"
                    value={cc.origin?.xyz?.[k] ?? 0}
                    onChange={(e) => {
                      const next = [...collisions];
                      const arr = [...(cc.origin?.xyz || [0, 0, 0])];
                      arr[k] = Number(e.target.value);
                      next[i] = { ...(cc || {}), origin: { ...(cc.origin || {}), xyz: arr } };
                      edit({ collisions: next });
                    }}
                  />
                ))}
              </div>
              <div className="rf-inline">
                <span>Origin rpy</span>
                {[0, 1, 2].map((k) => (
                  <input
                    key={k}
                    className="rf-input"
                    type="number"
                    step="any"
                    value={cc.origin?.rpy?.[k] ?? 0}
                    onChange={(e) => {
                      const next = [...collisions];
                      const arr = [...(cc.origin?.rpy || [0, 0, 0])];
                      arr[k] = Number(e.target.value);
                      next[i] = { ...(cc || {}), origin: { ...(cc.origin || {}), rpy: arr } };
                      edit({ collisions: next });
                    }}
                  />
                ))}
              </div>

              <button
                className="btn danger"
                onClick={() => {
                  const next = collisions.filter((_, j) => j !== i);
                  edit({ collisions: next });
                }}
              >
                Eliminar collision
              </button>
            </div>
          ))}

          <button
            className="btn"
            onClick={() =>
              edit({
                collisions: [
                  ...collisions,
                  { geometry: { type: "mesh", filename: "" }, origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] } },
                ],
              })
            }
          >
            + Collision
          </button>
        </details>
      </div>

     
      <Handle type="source" position={Position.Right} id="out" />
    </div>
  );
}
