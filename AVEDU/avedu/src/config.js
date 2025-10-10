// src/config.js (CRA-safe)
export const API_BASE =
  (typeof process !== "undefined" &&
    process.env &&
    process.env.REACT_APP_API_BASE) ||
  "http://localhost:8000/api";
