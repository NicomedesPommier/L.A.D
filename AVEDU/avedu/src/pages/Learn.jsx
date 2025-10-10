// =============================================================
// FILE: src/pages/Learn.jsx  (optimizado)
// =============================================================
import React, { useEffect, useMemo, useState, useCallback, useRef } from "react";
import { Outlet, NavLink, useParams } from "react-router-dom";
import { apiFetch } from "../context/AuthContext";
import { API_BASE } from "../config";
import { ProgressProvider } from "../context/ProgressContext";
import "../styles/pages/_learn.scss";

const DEBUG = true;

// --------- MERGE PROGRESS ----------
function mergeProgressIntoUnits(units, levelsProgress) {
  if (DEBUG) console.log("[mergeProgressIntoUnits] units.len =", units?.length, "levelsProgress.len =", levelsProgress?.length);

  const lvlMap = new Map((levelsProgress || []).map((l) => [l.slug, l]));

  return (units || []).map((u) => {
    const mergedLevels = (u.levels || []).map((l) => {
      const p = lvlMap.get(l.slug);
      if (!p) return l;

      const mergedLevel = {
        ...l,
        user_progress: p.user_progress || l.user_progress,
      };

      if (Array.isArray(l.objectives) && Array.isArray(p.objectives)) {
        const pObjMap = new Map(p.objectives.map((o) => [o.code, o]));
        mergedLevel.objectives = l.objectives.map((o) => {
          const po = pObjMap.get(o.code);
          return po ? { ...o, user_progress: po.user_progress } : o;
        });
      }
      return mergedLevel;
    });

    return { ...u, levels: mergedLevels };
  });
}

export default function Learn() {
  const [units, setUnits] = useState([]);
  const [loadingUnits, setLoadingUnits] = useState(true);
  const [, setLoadingProgress] = useState(false);

  const { unitSlug } = useParams();

  // Sidebar open/close (persisted)
  const [open, setOpen] = useState(() => localStorage.getItem("learn.sidebar.open") === "false" ? false : true);
  const toggle = useCallback(() => {
    setOpen((prev) => {
      const next = !prev;
      localStorage.setItem("learn.sidebar.open", String(next));
      return next;
    });
  }, []);
  const layoutClass = useMemo(() => `learn-layout${open ? "" : " learn-layout--collapsed"}`, [open]);

  // Abort refs
  const abortUnitsRef = useRef(null);
  const abortProgressRef = useRef(null);

  // --------- LOAD UNITS ----------
  const loadUnits = useCallback(async () => {
    if (abortUnitsRef.current) abortUnitsRef.current.abort();
    const ac = new AbortController();
    abortUnitsRef.current = ac;

    try {
      setLoadingUnits(true);
      if (DEBUG) console.log("[Learn] GET", `${API_BASE}/units/`);
      const res = await apiFetch(`${API_BASE}/units/`, { signal: ac.signal });
      if (DEBUG) console.log("[Learn] GET /units status:", res.status);
      const data = res.ok ? await res.json() : [];
      if (DEBUG) {
        console.log("[Learn] units summary:",
          (Array.isArray(data) ? data : []).map(u => ({
            unit: u.slug,
            levels: (u.levels || []).map(l => ({ slug: l.slug, objectives: (l.objectives || []).length }))
          }))
        );
      }
      setUnits(Array.isArray(data) ? data : []);
    } catch (e) {
      if (e?.name !== "AbortError") {
        console.error("[Learn] loadUnits error:", e);
        setUnits([]);
      }
    } finally {
      setLoadingUnits(false);
    }
  }, []);

  useEffect(() => { loadUnits(); }, [loadUnits]);

  // --------- FETCH PROGRESS ----------
  const fetchProgress = useCallback(async () => {
    if (abortProgressRef.current) abortProgressRef.current.abort();
    const ac = new AbortController();
    abortProgressRef.current = ac;

    const url = `${API_BASE}/levels/progress/me/`;
    try {
      setLoadingProgress(true);
      if (DEBUG) console.log("[Learn] GET", url);
      const res = await apiFetch(url, { signal: ac.signal });
      if (DEBUG) console.log("[Learn] GET /levels/progress/me/ status:", res.status);
      const data = res.ok ? await res.json() : [];
      if (DEBUG) {
        console.log("[Learn] levelsProgress summary:",
          (Array.isArray(data) ? data : []).map(l => ({
            slug: l.slug,
            objectives: (l.objectives || []).map(o => ({ code: o.code, achieved: o.user_progress?.achieved })),
            completed: l.user_progress?.completed,
            score: l.user_progress?.score
          }))
        );
      }
      return Array.isArray(data) ? data : [];
    } catch (e) {
      if (e?.name !== "AbortError") {
        console.error("[Learn] fetchProgress error:", e);
      }
      return [];
    } finally {
      setLoadingProgress(false);
    }
  }, []);

  const reloadProgress = useCallback(async () => {
    try {
      const levelsProgress = await fetchProgress();
      setUnits((prev) => mergeProgressIntoUnits(prev, levelsProgress));
    } catch (e) {
      console.error("[Learn] reloadProgress error:", e);
    }
  }, [fetchProgress]);

  // --------- ACTIVE UNIT ----------
  const activeUnit = useMemo(() => {
    const want = (unitSlug || "").toLowerCase();
    const found = units.find((u) => (u.slug || "").toLowerCase() === want) || null;
    if (DEBUG) console.log("[Learn] activeUnit want:", want, "found:", found?.slug);
    return found;
  }, [units, unitSlug]);

  const isUnitCompleted = useCallback((u) => {
    const levels = Array.isArray(u?.levels) ? u.levels : [];
    return levels.length > 0 && levels.every((l) => !!l.user_progress?.completed);
  }, []);

  if (DEBUG) {
    console.log("[Learn] render snapshot units:",
      units.map(u => ({
        unit: u.slug,
        levels: (u.levels || []).map(l => ({
          slug: l.slug,
          completed: l.user_progress?.completed,
          obj: (l.objectives || []).map(o => ({ code: o.code, done: o.user_progress?.achieved }))
        }))
      }))
    );
  }

  // --------- RENDER ----------
  return (
  <ProgressProvider>
    <div className={layoutClass}>
      {/* Toggle flotante SIEMPRE montado.
          Tu SCSS ya lo oculta cuando no está colapsado. */}
      <button
        className="learn-toggle learn-toggle--floating"
        onClick={toggle}
        aria-label={open ? "Ocultar barra lateral" : "Mostrar barra lateral"}
        title={open ? "Ocultar" : "Mostrar"}
      >
        {open ? "⟨" : "⟩"}
      </button>

      {/* Sidebar */}
      <aside className="learn-sidebar">
        <div className="learn-sidebar__header">
          <h2>{activeUnit ? activeUnit.title : "Units"}</h2>
          {/* Puedes dejar este botón interno también */}
          <button
            className="learn-toggle"
            onClick={toggle}
            title={open ? "Hide" : "Show"}
          >
            {open ? "⟨" : "⟩"}
          </button>
        </div>

        {loadingUnits ? (
          <div className="learn-sidebar__loading">Cargando…</div>
        ) : activeUnit ? (
          <ul>
            {(activeUnit.levels || []).map((l) => (
              <li key={l.slug}>
                <NavLink
                  to={`/learn/${activeUnit.slug}/${l.slug}`}
                  className={({ isActive }) => (isActive ? "active" : undefined)}
                >
                  {l.title}
                </NavLink>
                {l.user_progress?.completed && <span className="badge">✔</span>}
              </li>
            ))}
          </ul>
        ) : (
          <ul>
            {units.map((u) => (
              <li key={u.slug}>
                <NavLink
                  to={`/learn/${u.slug}`}
                  className={({ isActive }) => (isActive ? "active" : undefined)}
                >
                  {u.title}
                </NavLink>
                {isUnitCompleted(u) && <span className="badge">✔</span>}
              </li>
            ))}
          </ul>
        )}
      </aside>

      {/* Stage */}
      <main className="learn-stage">
        {loadingUnits || units.length === 0 ? (
          <div className="loading">Cargando unidades…</div>
        ) : (
          <Outlet context={{ units, reloadProgress }} />
        )}
      </main>
    </div>
  </ProgressProvider>
)
}
