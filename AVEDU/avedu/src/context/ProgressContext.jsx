// src/context/ProgressContext.jsx
import React, { createContext, useContext, useMemo } from "react";
import { apiFetch } from "./AuthContext";
import { API_BASE } from "../config";

const Ctx = createContext(null);
export const useProgress = () => useContext(Ctx);

export function ProgressProvider({ children }) {
  async function hitObjective(code) {
    const url = `${API_BASE}/progress/objective/${encodeURIComponent(code)}/hit/`;
    console.log("[Progress] hitObjective ->", url);
    const res = await apiFetch(url, { method: "POST" });
    const text = await res.text();
    let body = null; try { body = text ? JSON.parse(text) : null; } catch {}
    console.log("[Progress] hitObjective status:", res.status, "body:", body ?? text);
    if (!res.ok) throw new Error(`hitObjective failed (${res.status})`);
    return body;
  }

  async function completeLevel(slug) {
    const url = `${API_BASE}/levels/${encodeURIComponent(slug)}/complete/`;
    console.log("[Progress] completeLevel ->", url);
    const res = await apiFetch(url, { method: "POST" });
    const text = await res.text();
    let body = null; try { body = text ? JSON.parse(text) : null; } catch {}
    console.log("[Progress] completeLevel status:", res.status, "body:", body ?? text);
    if (!res.ok) throw new Error(`completeLevel failed (${res.status})`);
    return body;
  }

  async function resetLevel(slug) {
    const url = `${API_BASE}/levels/${encodeURIComponent(slug)}/reset/`;
    console.log("[Progress] resetLevel ->", url);
    const res = await apiFetch(url, { method: "POST" });
    const text = await res.text();
    let body = null; try { body = text ? JSON.parse(text) : null; } catch {}
    console.log("[Progress] resetLevel status:", res.status, "body:", body ?? text);
    if (!res.ok) throw new Error(`resetLevel failed (${res.status})`);
    return body;
  }

  const value = useMemo(() => ({ hitObjective, completeLevel, resetLevel }), []);
  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}
