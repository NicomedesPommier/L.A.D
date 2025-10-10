// src/ip.js
import React from "react";

/** Config base (puedes cambiar los puertos si quieres otros defaults) */
const DEFAULTS = {
  HOST: "localhost",
  PORTS: {
    ROSBRIDGE: 9090,  // ws(s)://HOST:9090
    STATIC: 7000,     // http(s)://HOST:7000
    API: 8000,        // http(s)://HOST:8000
    WVS: 8080,        // http(s)://HOST:8080  (si lo usas)
  },
};

/** Estado module-level (single source of truth) */
let CURRENT_HOST = process.env.REACT_APP_HOST || DEFAULTS.HOST;
let CURRENT_PORTS = {
  ...DEFAULTS.PORTS,
  ...(process.env.REACT_APP_ROSBRIDGE_PORT ? { ROSBRIDGE: +process.env.REACT_APP_ROSBRIDGE_PORT } : {}),
  ...(process.env.REACT_APP_STATIC_PORT ? { STATIC: +process.env.REACT_APP_STATIC_PORT } : {}),
  ...(process.env.REACT_APP_API_PORT ? { API: +process.env.REACT_APP_API_PORT } : {}),
  ...(process.env.REACT_APP_WVS_PORT ? { WVS: +process.env.REACT_APP_WVS_PORT } : {}),
};

/** Si pasas ?host=IP o ?ros=ws://IP:9090 en la URL, lo respeta (útil en laboratorio) */
try {
  const qs = new URLSearchParams(window.location.search);
  const hostQs = qs.get("host");
  if (hostQs) CURRENT_HOST = hostQs;

  const rosQs = qs.get("ros"); // p.ej. ws://10.0.0.5:9090
  if (rosQs) {
    const u = new URL(rosQs);
    CURRENT_HOST = u.hostname || CURRENT_HOST;
    if (u.port) CURRENT_PORTS.ROSBRIDGE = +u.port;
  }
} catch {}

/** Detecta http/https y ajusta ws/wss automáticamente */
const isHttps = () => window.location?.protocol === "https:";
const wsScheme = () => (isHttps() ? "wss" : "ws");
const httpScheme = () => (isHttps() ? "https" : "http");

/** Helpers genéricos */
const makeWs = (port, path = "") => `${wsScheme()}://${CURRENT_HOST}:${port}${path}`;
const makeHttp = (port, path = "") => `${httpScheme()}://${CURRENT_HOST}:${port}${path.replace(/^\//, "/")}`;

/** Getters específicos (puertos que usas) */
export const getRosbridgeUrl = (path = "") => makeWs(CURRENT_PORTS.ROSBRIDGE, path);   // ws(s)://HOST:9090
export const getStaticBase = () => makeHttp(CURRENT_PORTS.STATIC);                     // http(s)://HOST:7000
export const getApiBase = () => makeHttp(CURRENT_PORTS.API);                           // http(s)://HOST:8000
export const getWvsBase = () => makeHttp(CURRENT_PORTS.WVS);                           // http(s)://HOST:8080

/** Componente “una línea”: fija HOST y puertos para toda la app */
export function IP({ host, ports = {}, children }) {
  if (host) CURRENT_HOST = host;
  CURRENT_PORTS = { ...CURRENT_PORTS, ...ports };
  return <>{children}</>;
}
