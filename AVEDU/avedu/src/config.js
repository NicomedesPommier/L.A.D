// src/config.js
export const API_BASE = process.env.REACT_APP_API_BASE || '/api';
export const STATIC_BASE = '/qcar_static'; // opcional si la usas en más lados

// Check if running in Kubernetes (path-based routing via ingress)
// When PUBLIC_URL is set (e.g., /lad), use ingress paths instead of ports
const USE_INGRESS = process.env.PUBLIC_URL && process.env.PUBLIC_URL !== '/';
const INGRESS_BASE = process.env.PUBLIC_URL || '';

// ROS integration - uses ingress paths in K8s, localhost ports in local dev
export const ROS_STATIC_BASE = process.env.REACT_APP_ROS_STATIC ||
  (USE_INGRESS ? `${INGRESS_BASE}/ros-static` : 'http://localhost:7000');

export const ROS_WS_URL = process.env.REACT_APP_ROS_WS ||
  (USE_INGRESS
    ? `${window.location.protocol === 'https:' ? 'wss' : 'ws'}://${window.location.host}${INGRESS_BASE}/rosbridge`
    : 'ws://localhost:9090');
