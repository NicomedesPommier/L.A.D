// =============================================================
// FILE: src/context/AuthContext.jsx
// =============================================================
import React, { createContext, useContext, useMemo, useState, useCallback } from "react";
import { API_BASE } from "../config";

const AuthContext = createContext();
export const useAuth = () => useContext(AuthContext);

export default function AuthProvider({ children }) {
  const [token, setToken] = useState(() => localStorage.getItem("token") || null);
  const [user, setUser] = useState(() =>
    token ? { username: localStorage.getItem("username") || "user" } : null
  );
  const [loading, setLoading] = useState(false);

  // Normaliza base: sin slash final
  const apiRoot = API_BASE.replace(/\/$/, "");

  const saveSession = useCallback(({ access, username }) => {
    console.log("[Auth] saveSession access(head):", (access || "").slice(0, 20) + "...");
    console.log("[Auth] username:", username);
    localStorage.setItem("token", access);
    if (username) localStorage.setItem("username", username);
    setToken(access);
    setUser({ username: username || "user" });
  }, []);

  const login = useCallback(
    async (username, password) => {
      setLoading(true);
      try {
        const loginUrl = `${apiRoot}/token/`; // típicamente => http://host/api/token/
        console.log("[Auth] login fetch ->", loginUrl);
        const res = await fetch(loginUrl, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ username, password }),
        });

        console.log("[Auth] /token/ status:", res.status);
        if (!res.ok) {
          const text = await res.text().catch(() => "");
          console.log("[Auth] /token/ body:", text);
          throw new Error("Invalid credentials");
        }

        const data = await res.json(); // { access, refresh }
        console.log("[Auth] token(access) head:", (data?.access || "").slice(0, 20) + "...");
        saveSession({ access: data.access, username });
        return true;
      } finally {
        setLoading(false);
      }
    },
    [apiRoot, saveSession]
  );

  const logout = useCallback(() => {
    console.log("[Auth] logout()");
    localStorage.removeItem("token");
    localStorage.removeItem("username");
    setToken(null);
    setUser(null);
  }, []);

  const register = useCallback(
    async ({ username, password, firstName, lastName, email }) => {
      setLoading(true);
      try {
        const registerUrl = `${apiRoot}/register/`; // típicamente => http://host/api/register/
        console.log("[Auth] register fetch ->", registerUrl);
        const res = await fetch(registerUrl, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            username,
            password,
            first_name: firstName,
            last_name: lastName,
            email,
          }),
        });

        if (!res.ok) {
          let detail = "No se pudo crear el usuario";
          try {
            const data = await res.json();
            if (typeof data === "string") detail = data;
            else if (data?.detail) detail = data.detail;
            else if (typeof data === "object") {
              detail = Object.entries(data)
                .map(([key, value]) => `${key}: ${Array.isArray(value) ? value.join(", ") : value}`)
                .join("\n");
            }
          } catch {
            /* ignore parse error */
          }
          throw new Error(detail);
        }

        await login(username, password);
        return true;
      } finally {
        setLoading(false);
      }
    },
    [apiRoot, login]
  );

  const value = useMemo(
    () => ({ token, user, login, logout, register, loading }),
    [token, user, login, logout, register, loading]
  );

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

// =============================================================
// Helper para fetch con token y logs
// =============================================================
export async function apiFetch(url, options = {}) {
  const t = localStorage.getItem("token");
  const headers = { ...(options.headers || {}), ...(t ? { Authorization: `Bearer ${t}` } : {}) };

  // 🔎 REQUEST
  console.log("[apiFetch] →", options.method || "GET", url);
  console.log("[apiFetch] headers:", {
    ...headers,
    Authorization: headers.Authorization ? headers.Authorization.slice(0, 30) + "..." : undefined,
  });
  if (options.body) {
    try {
      console.log(
        "[apiFetch] body:",
        typeof options.body === "string" ? options.body : JSON.stringify(options.body)
      );
    } catch {
      /* ignore */
    }
  }

  const res = await fetch(url, { ...options, headers });

  // 🔎 RESPONSE (clon para no consumir stream)
  const clone = res.clone();
  let text = "";
  try {
    text = await clone.text();
  } catch {
    /* ignore */
  }
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
