// src/config.js
export const API_BASE = process.env.REACT_APP_API_BASE || '/api';
export const STATIC_BASE = '/qcar_static'; // opcional si la usas en más lados

// ROS integration - uses ingress paths for Kubernetes deployment
export const ROS_STATIC_BASE = process.env.REACT_APP_ROS_STATIC || '/lad/ros-static';
export const ROS_WS_URL =
  process.env.REACT_APP_ROS_WS ||
  `${window.location.protocol === 'https:' ? 'wss' : 'ws'}://${window.location.host}/lad/rosbridge`;
