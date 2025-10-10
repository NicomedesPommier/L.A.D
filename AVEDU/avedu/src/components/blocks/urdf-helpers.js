// components/blocks/urdf-helpers.js
// Helpers puros para serializar URDF y derivar XML desde el grafo (@xyflow/react)

/** Escapa caracteres XML */
export const esc = (s = "") =>
  String(s)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&apos;");

/** Formatea vectores xyz/rpy a "x y z" */
export const fmtVec = (v) => (Array.isArray(v) ? v.join(" ") : String(v ?? ""));

/** <visual> ... </visual> */
export function visualToXml(v) {
  if (!v) return "";
  const origin = v.origin
    ? `<origin xyz="${esc(fmtVec(v.origin.xyz))}" rpy="${esc(fmtVec(v.origin.rpy))}"/>`
    : "";
  let geom = "";
  if (v.geometry?.type === "mesh") {
    const scale = v.geometry.scale ? ` scale="${esc(fmtVec(v.geometry.scale))}"` : "";
    geom = `<geometry><mesh filename="${esc(v.geometry.filename || "")}"${scale}/></geometry>`;
  } else if (v.geometry?.type === "box") {
    geom = `<geometry><box size="${esc(fmtVec(v.geometry.size || [1, 1, 1]))}"/></geometry>`;
  } else if (v.geometry?.type === "cylinder") {
    geom = `<geometry><cylinder radius="${esc(v.geometry.radius || 0.05)}" length="${esc(
      v.geometry.length || 0.1
    )}"/></geometry>`;
  } else if (v.geometry?.type === "sphere") {
    geom = `<geometry><sphere radius="${esc(v.geometry.radius || 0.05)}"/></geometry>`;
  }
  const material = v.material?.name ? `<material name="${esc(v.material.name)}"/>` : "";
  return `<visual>${origin}${geom}${material}</visual>`;
}

/** <collision> ... </collision> */
export function collisionToXml(c) {
  if (!c) return "";
  const origin = c.origin
    ? `<origin xyz="${esc(fmtVec(c.origin.xyz))}" rpy="${esc(fmtVec(c.origin.rpy))}"/>`
    : "";
  let geom = "";
  if (c.geometry?.type === "mesh") {
    const scale = c.geometry.scale ? ` scale="${esc(fmtVec(c.geometry.scale))}"` : "";
    geom = `<geometry><mesh filename="${esc(c.geometry.filename || "")}"${scale}/></geometry>`;
  } else if (c.geometry?.type === "box") {
    geom = `<geometry><box size="${esc(fmtVec(c.geometry.size || [1, 1, 1]))}"/></geometry>`;
  } else if (c.geometry?.type === "cylinder") {
    geom = `<geometry><cylinder radius="${esc(c.geometry.radius || 0.05)}" length="${esc(
      c.geometry.length || 0.1
    )}"/></geometry>`;
  } else if (c.geometry?.type === "sphere") {
    geom = `<geometry><sphere radius="${esc(c.geometry.radius || 0.05)}"/></geometry>`;
  }
  return `<collision>${origin}${geom}</collision>`;
}

/** <inertial> ... </inertial> */
export function inertialToXml(i) {
  if (!i) return "";
  const origin = i.origin
    ? `<origin xyz="${esc(fmtVec(i.origin.xyz))}" rpy="${esc(fmtVec(i.origin.rpy))}"/>`
    : "";
  const mass =
    i.mass || i.mass === 0 ? `<mass value="${esc(i.mass)}"/>` : "";
  const I = i.inertia || {};
  const inertia =
    I.ixx || I.iyy || I.izz
      ? `<inertia ixx="${esc(I.ixx || 0)}" ixy="${esc(I.ixy || 0)}" ixz="${esc(
          I.ixz || 0
        )}" iyy="${esc(I.iyy || 0)}" iyz="${esc(I.iyz || 0)}" izz="${esc(I.izz || 0)}"/>`
      : "";
  return `<inertial>${origin}${mass}${inertia}</inertial>`;
}

/** <link> ... </link> */
export function linkToXml(l) {
  if (!l?.name) return "";
  const visuals = Array.isArray(l.visuals) ? l.visuals.map(visualToXml).join("") : "";
  const collisions = Array.isArray(l.collisions)
    ? l.collisions.map(collisionToXml).join("")
    : "";
  const inertial = inertialToXml(l.inertial);
  return `<link name="${esc(l.name)}">${inertial}${visuals}${collisions}</link>`;
}

/** <joint> ... </joint> */
export function jointToXml(j) {
  if (!j?.name || !j.parent || !j.child || !j.type) return "";
  const origin = j.origin
    ? `<origin xyz="${esc(fmtVec(j.origin.xyz))}" rpy="${esc(fmtVec(j.origin.rpy))}"/>`
    : "";
  const axis = j.axis ? `<axis xyz="${esc(fmtVec(j.axis.xyz))}"/>` : "";
  return `<joint name="${esc(j.name)}" type="${esc(j.type)}">${origin}<parent link="${esc(
    j.parent
  )}"/><child link="${esc(j.child)}"/>${axis}</joint>`;
}

/** Documento <robot>… con links y joints */
export function robotToXml(name, links = [], joints = []) {
  const linkXml = links.map(linkToXml).filter(Boolean).join("\n");
  const jointXml = joints.map(jointToXml).filter(Boolean).join("\n");
  return `<?xml version="1.0"?>\n<robot name="${esc(name || "my_robot")}">\n${linkXml}\n${jointXml}\n</robot>`;
}

/* ---------- Derivadas desde el grafo de React Flow ---------- */

/** Extrae del grafo los datos conectados al nodo urdfRobot */
export function computeRobotData(nodes, edges) {
  const robot = nodes.find((n) => n.type === "urdfRobot");
  if (!robot) return { robotId: null, links: [], joints: [], name: "my_robot" };

  const incoming = edges.filter((e) => e.target === robot.id);
  const fromPort = (id) =>
    incoming
      .filter((e) => e.targetHandle === id)
      .map((e) => nodes.find((n) => n.id === e.source)?.data)
      .filter(Boolean);

  const links = fromPort("links").map((d) => ({ ...d }));
  const joints = fromPort("joints").map((d) => ({ ...d }));
  const name = robot.data?.name || "my_robot";
  return { robotId: robot.id, links, joints, name };
}

/** Genera el XML del robot actualmente conectado */
export function computeUrdfXml(nodes, edges) {
  const { robotId, links, joints, name } = computeRobotData(nodes, edges);
  if (!robotId) return { xml: "", robotId: null };
  const xml = robotToXml(name, links, joints);
  return { xml, robotId };
}

/**
 * Sincroniza el XML generado hacia:
 *  - el propio nodo urdfRobot (data.xml)
 *  - cualquier urdfPreview / urdfViewer en el grafo
 */
// urdf-helpers.js
export function syncUrdfDerived(nodes, edges, setNodes) {
  const deriv = computeUrdfXml(nodes, edges);
  if (!deriv.robotId) return;
  const { xml, robotId } = deriv;

  setNodes(prev => {
    let changed = false;
    const next = prev.map(n => {
      const curr = n.data || {};
      const isTarget = (n.id === robotId) || (n.type === 'urdfPreview') || (n.type === 'urdfViewer');
      if (!isTarget) return n;

      if (curr.xml === xml) return n; // nada cambia
      changed = true;
      return { ...n, data: { ...curr, xml } };
    });

    
    return changed ? next : prev;
  });
}
