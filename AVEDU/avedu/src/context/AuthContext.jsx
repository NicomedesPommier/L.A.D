// =============================================================
// FILE: src/context/AuthContext.jsx (fix deps)
// =============================================================
import React, { createContext, useContext, useMemo, useState, useCallback } from "react";

const AuthContext = createContext();
export const useAuth = () => useContext(AuthContext);

export default function AuthProvider({ children }) {
  const [token, setToken] = useState(() => localStorage.getItem("token") || null);
  const [user, setUser] = useState(() =>
    token ? { username: localStorage.getItem("username") || "user" } : null
  );
  const [loading, setLoading] = useState(false);

  const saveSession = useCallback(({ access, username }) => {
    console.log("[Auth] saveSession access(head):", (access || "").slice(0, 20) + "...");
    console.log("[Auth] username:", username);
    localStorage.setItem("token", access);
    if (username) localStorage.setItem("username", username);
    setToken(access);
    setUser({ username: username || "user" });
  }, []);

  const login = useCallback(async (username, password) => {
    setLoading(true);
    try {
      console.log("[Auth] login fetch -> /api/token/");
      const res = await fetch("http://localhost:8000/api/token/", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ username, password }),
      });
      console.log("[Auth] /api/token/ status:", res.status);
      if (!res.ok) {
        const text = await res.text().catch(() => "");
        console.log("[Auth] /api/token/ body:", text);
        throw new Error("Invalid credentials");
      }
      const data = await res.json(); // { access, refresh }
      console.log("[Auth] token(access) head:", (data?.access || "").slice(0, 20) + "...");
      saveSession({ access: data.access, username });
      return true;
    } finally {
      setLoading(false);
    }
  }, [saveSession]);

  const logout = useCallback(() => {
    console.log("[Auth] logout()");
    localStorage.removeItem("token");
    localStorage.removeItem("username");
    setToken(null);
    setUser(null);
  }, []);

  const value = useMemo(
    () => ({ token, user, login, logout, loading }),
    [token, user, login, logout, loading]
  );

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}


// Attach token to all fetches (optional helper)
// Attach token to all fetches (improved logger)
export async function apiFetch(url, options = {}) {
  const t = localStorage.getItem("token");
  const headers = { ...(options.headers || {}), ...(t ? { Authorization: `Bearer ${t}` } : {}) };

  // 🔎 REQUEST
  console.log("[apiFetch] →", (options.method || "GET"), url);
  console.log("[apiFetch] headers:", {
    ...headers,
    Authorization: headers.Authorization ? headers.Authorization.slice(0, 30) + "..." : undefined,
  });
  if (options.body) {
    try {
      console.log("[apiFetch] body:", typeof options.body === "string" ? options.body : JSON.stringify(options.body));
    } catch {}
  }

  const res = await fetch(url, { ...options, headers });

  // 🔎 RESPONSE (sin consumir el stream original)
  const clone = res.clone();
  let text = "";
  try { text = await clone.text(); } catch {}

  console.log("[apiFetch] ←", res.status, res.statusText, url);
  if (text) {
    try {
      const json = JSON.parse(text);
      console.log("[apiFetch] JSON:", json);
    } catch {
      console.log("[apiFetch] TEXT:", text.slice(0, 2000));
    }
  } else {
    console.log("[apiFetch] (empty body)");
  }

  return res;
}
