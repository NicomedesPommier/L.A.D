// components/blocks/index.js
// Registro de nodos + paletas + defaultData para tus slides

import CreatePackageNode from "./CreatePackageNode";
import RosRunNode from "./RosRunNode";
import StringNode from "./StringNode";
import ListItemsNode from "./ListItemsNode";
import ConvertToCodeNode from "./ConvertToCodeNode";

// --- URDF blocks (nuevos) ---
import UrdfLinkNode from "./UrdfLinkNode";
import UrdfJointNode from "./UrdfJointNode";
import UrdfRobotNode from "./UrdfRobotNode";
import UrdfPreviewNode from "./UrdfPreviewNode";
import UrdfViewerNode from "./UrdfViewerNode";

export const nodeTypes = {
  // existentes
  createPackage: CreatePackageNode,
  rosRun: RosRunNode,
  string: StringNode,
  listArgs: ListItemsNode,
  listDeps: ListItemsNode,
  toCode: ConvertToCodeNode,

  // nuevos URDF
  urdfLink: UrdfLinkNode,
  urdfJoint: UrdfJointNode,
  urdfRobot: UrdfRobotNode,
  urdfPreview: UrdfPreviewNode,
  urdfViewer: UrdfViewerNode,
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

// Nueva paleta: URDF
export const paletteUrdf = [
  { type: "urdfLink",    label: "URDF Link" },
  { type: "urdfJoint",   label: "URDF Joint" },
  { type: "urdfRobot",   label: "URDF Robot" },
  { type: "urdfPreview", label: "URDF XML" },
  { type: "urdfViewer",  label: "URDF Viewer" },
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

  return {};
}
