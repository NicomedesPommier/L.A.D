import React, { useState } from "react";
import { Handle, Position } from "@xyflow/react";

const DEFAULT_DEPS = {
  python: ["rclpy", "std_msgs"],
  cpp: ["rclcpp", "std_msgs"],
};

export default function CreatePackageNode({ id, data }) {
  const [pkgName, setPkgName] = useState(data.pkgName ?? "my_ros2_package");
  const [nodeName, setNodeName] = useState(data.nodeName ?? "my_node");
  const [lang, setLang] = useState(data.lang ?? "python"); // 'python' | 'cpp'
  const [buildType, setBuildType] = useState(
    data.buildType ?? (lang === "cpp" ? "ament_cmake" : "ament_python")
  );
  const [depInput, setDepInput] = useState("");
  const [deps, setDeps] = useState(
    Array.isArray(data.deps) && data.deps.length ? data.deps : DEFAULT_DEPS[lang]
  );

  const notify = (next) => data.onChange?.(id, { ...next });

  const onLangChange = (value) => {
    setLang(value);
    const defaultBT = value === "cpp" ? "ament_cmake" : "ament_python";
    setBuildType((prev) =>
      prev === "ament_cmake" || prev === "ament_python" ? defaultBT : prev
    );
    setDeps((prev) => (prev && prev.length ? prev : DEFAULT_DEPS[value]));
    notify({
      pkgName,
      nodeName,
      lang: value,
      buildType: defaultBT,
      deps: deps && deps.length ? deps : DEFAULT_DEPS[value],
    });
  };

  const onBuildTypeChange = (v) => {
    setBuildType(v);
    notify({ pkgName, nodeName, lang, buildType: v, deps });
  };
  const onPkgChange = (v) => {
    setPkgName(v);
    notify({ pkgName: v, nodeName, lang, buildType, deps });
  };
  const onNodeChange = (v) => {
    setNodeName(v);
    notify({ pkgName, nodeName: v, lang, buildType, deps });
  };

  const addDep = () => {
    const parts = depInput
      .split(/[,\s]+/)
      .map((s) => s.trim())
      .filter(Boolean);
    if (!parts.length) return;
    const next = Array.from(new Set([...(deps || []), ...parts]));
    setDeps(next);
    setDepInput("");
    notify({ pkgName, nodeName, lang, buildType, deps: next });
  };
  const removeDep = (d) => {
    const next = (deps || []).filter((x) => x !== d);
    setDeps(next);
    notify({ pkgName, nodeName, lang, buildType, deps: next });
  };

  return (
    <div className="rf-card">
      <div className="rf-card__title">Create Package</div>

      <div className="rf-card__body">
        <label className="rf-field">
          <span>Package name</span>
          <input
            value={pkgName}
            onChange={(e) => onPkgChange(e.target.value)}
            placeholder="my_ros2_package"
            className="rf-input"
          />
        </label>

        <label className="rf-field">
          <span>Node name</span>
          <input
            value={nodeName}
            onChange={(e) => onNodeChange(e.target.value)}
            placeholder="my_node"
            className="rf-input"
          />
        </label>

        <div className="rf-grid-2">
          <label className="rf-field">
            <span>Language</span>
            <select
              value={lang}
              onChange={(e) => onLangChange(e.target.value)}
              className="rf-input"
            >
              <option value="python">Python</option>
              <option value="cpp">C++</option>
            </select>
          </label>

          <label className="rf-field">
            <span>Build type</span>
            <select
              value={buildType}
              onChange={(e) => onBuildTypeChange(e.target.value)}
              className="rf-input"
            >
              <option value="ament_python">ament_python</option>
              <option value="ament_cmake">ament_cmake</option>
            </select>
          </label>
        </div>

        <div className="rf-field">
          <span>Dependencies</span>
          <div className="rf-row">
            <input
              value={depInput}
              onChange={(e) => setDepInput(e.target.value)}
              placeholder="rclpy, std_msgs"
              className="rf-input"
            />
            <button className="btn" onClick={addDep}>agregar</button>
          </div>
          <div className="rf-chips">
            {(deps || []).map((d) => (
              <span key={d} className="rf-chip" onClick={() => removeDep(d)} title="Eliminar">
                {d} ✕
              </span>
            ))}
            {(!deps || deps.length === 0) && (
              <span className="rf-chip rf-chip--ghost">Sin deps</span>
            )}
          </div>
        </div>
      </div>

      {/* Entradas desde string/deps */}
      <Handle type="target" position={Position.Left} id="pkgName" style={{ top: "40%" }} />
      <Handle type="target" position={Position.Left} id="nodeName" style={{ top: "60%" }} />
      <Handle type="target" position={Position.Left} id="deps" style={{ top: "80%" }} />

      {/* Salida hacia Convert2Code (u otros) */}
      <Handle type="source" position={Position.Right} id="out" style={{ top: "50%" }} />
    </div>
  );
}
