// components/blocks/index.js
// Registro de nodos + paletas + defaultData para tus slides

import CreatePackageNode from "./CreatePackageNode";
import RosRunNode from "./RosRunNode";
import StringNode from "./StringNode";
import ListItemsNode from "./ListItemsNode";
import ConvertToCodeNode from "./ConvertToCodeNode";

// --- URDF blocks (original) ---
import UrdfLinkNode from "./UrdfLinkNode";
import UrdfJointNode from "./UrdfJointNode";
import UrdfRobotNode from "./UrdfRobotNode";
import UrdfPreviewNode from "./UrdfPreviewNode";
import UrdfViewerNode from "./UrdfViewerNode";

// --- URDF blocks V2 (modular/improved) ---
import UrdfLinkNodeV2 from "./UrdfLinkNodeV2";
import UrdfInertialNode from "./UrdfInertialNode";
import UrdfVisualNode from "./UrdfVisualNode";
import UrdfCollisionNode from "./UrdfCollisionNode";
import UrdfAssemblyNode from "./UrdfAssemblyNode";
import UrdfXmlPreviewNode from "./UrdfXmlPreviewNode";

export const nodeTypes = {
  // existentes
  createPackage: CreatePackageNode,
  rosRun: RosRunNode,
  string: StringNode,
  listArgs: ListItemsNode,
  listDeps: ListItemsNode,
  toCode: ConvertToCodeNode,

  // URDF (original)
  urdfLink: UrdfLinkNode,
  urdfJoint: UrdfJointNode,
  urdfRobot: UrdfRobotNode,
  urdfPreview: UrdfPreviewNode,
  urdfViewer: UrdfViewerNode,

  // URDF V2 (modular/improved)
  urdfLinkV2: UrdfLinkNodeV2,
  urdfInertial: UrdfInertialNode,
  urdfVisual: UrdfVisualNode,
  urdfCollision: UrdfCollisionNode,
  urdfAssembly: UrdfAssemblyNode,
  urdfXmlPreview: UrdfXmlPreviewNode,
};

// Paleta existente: run
export const paletteRun = [
  { type: "string:pkg", label: "Package" },
  { type: "string:exe", label: "Executable" },
  { type: "string:ns",  label: "Namespace" },
  { type: "listArgs",   label: "Args" },
  { type: "rosRun",     label: "ROS Run" },
  { type: "toCode",     label: "Convert2Code" },
];

// Paleta existente: create package
export const paletteCreate = [
  { type: "string:pkgName", label: "Package Name" },
  { type: "string:nodeName", label: "Node Name" },
  { type: "listDeps", label: "Dependencies" },
  { type: "createPackage", label: "Create Package" },
  { type: "toCode", label: "Convert2Code" },
];

// Paleta URDF (Original - all-in-one)
export const paletteUrdf = [
  { type: "urdfLink",    label: "URDF Link" },
  { type: "urdfJoint",   label: "URDF Joint" },
  { type: "urdfRobot",   label: "URDF Robot" },
  { type: "urdfPreview", label: "URDF XML" },
  { type: "urdfViewer",  label: "URDF Viewer" },
];

// Paleta URDF V2 (Modular - Blender-style)
export const paletteUrdfV2 = [
  // Component nodes
  { type: "urdfInertial",  label: "⚖️ Inertial" },
  { type: "urdfVisual",    label: "👁️ Visual" },
  { type: "urdfCollision", label: "🛡️ Collision" },

  // Structure nodes
  { type: "urdfLinkV2",    label: "🔗 Link" },
  { type: "urdfJoint",     label: "🔩 Joint" },
  { type: "urdfAssembly",  label: "🔧 Assembly" },

  // Output nodes
  { type: "urdfRobot",      label: "🤖 Robot" },
  { type: "urdfXmlPreview", label: "📄 XML Preview" },
  { type: "urdfViewer",     label: "👀 3D Viewer" },
];

export function defaultDataFor(typeOrPreset) {
  // -------- existentes --------
  if (typeOrPreset === "rosRun")
    return { pkg: "", exe: "", ns: "", args: [] };

  if (typeOrPreset === "string:pkg")
    return { label: "Package", value: "turtlesim", placeholder: "turtlesim" };
  if (typeOrPreset === "string:exe")
    return { label: "Executable", value: "turtlesim_node", placeholder: "turtlesim_node" };
  if (typeOrPreset === "string:ns")
    return { label: "Namespace", value: "", placeholder: "/demo (opcional)" };

  if (typeOrPreset === "string:pkgName")
    return { label: "Package Name", value: "my_ros2_package" };
  if (typeOrPreset === "string:nodeName")
    return { label: "Node Name", value: "my_node" };

  if (typeOrPreset === "listArgs")
    return { title: "Args", keyName: "args", items: [], placeholder: "--ros-args ..." };
  if (typeOrPreset === "listDeps")
    return { title: "Dependencies", keyName: "deps", items: [], placeholder: "rclpy" };

  if (typeOrPreset === "createPackage") {
    return {
      pkgName: "my_ros2_package",
      nodeName: "my_node",
      lang: "python",
      buildType: "ament_python",
      deps: ["rclpy", "std_msgs"],
    };
  }

  if (typeOrPreset === "toCode")
    return { inCount: 0, preview: "" };

  // -------- nuevos URDF --------
  if (typeOrPreset === "urdfLink")
    return {
      id: "",
      name: "",
      visuals: [],
      collisions: [],
      inertial: {
        mass: 1,
        inertia: { ixx: 0.01, iyy: 0.01, izz: 0.01 },
        origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
      },
    };

  if (typeOrPreset === "urdfJoint")
    return {
      id: "",
      name: "",
      type: "fixed",
      parent: "",
      child: "",
      origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
      axis: { xyz: [1, 0, 0] },
    };

  if (typeOrPreset === "urdfRobot")
    return { id: "", name: "my_robot", links: [], joints: [], xml: "" };

  if (typeOrPreset === "urdfPreview")
    return { id: "", xml: "" };

  if (typeOrPreset === "urdfViewer")
    return { id: "", xml: "" };

  // -------- URDF V2 (modular) --------
  if (typeOrPreset === "urdfInertial")
    return {
      mass: 1.0,
      inertia: { ixx: 0.01, iyy: 0.01, izz: 0.01, ixy: 0, ixz: 0, iyz: 0 },
      origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] }
    };

  if (typeOrPreset === "urdfVisual")
    return {
      geometry: { type: "box", size: [1, 1, 1] },
      origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
      material: { name: "", color: [0.5, 0.5, 0.5, 1] }
    };

  if (typeOrPreset === "urdfCollision")
    return {
      geometry: { type: "box", size: [1, 1, 1] },
      origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] }
    };

  if (typeOrPreset === "urdfLinkV2")
    return {
      name: "",
      inertial: null,
      visuals: [],
      collisions: []
    };

  if (typeOrPreset === "urdfAssembly")
    return {
      name: "",
      description: "",
      links: [],
      joints: []
    };

  if (typeOrPreset === "urdfXmlPreview")
    return { xml: "" };

  return {};
}
