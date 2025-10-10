// =============================================================
// FILE: src/pages/Home.jsx
// Pantalla de inicio + login gate: si no hay sesión, muestra login.
// =============================================================
import React, { useEffect, useRef, useState } from "react";
import { useNavigate } from "react-router-dom";
import { useAuth } from "../context/AuthContext";
import "../styles/pages/_home.scss";

export default function Home() {
  const navigate = useNavigate();
  const { user, login, logout, loading } = useAuth();
  const startRef = useRef(null);

  useEffect(() => { startRef.current?.focus(); }, [user]);

  function handleStart() { navigate("/Learn"); }

  return (
    <div className="screen">
      <div className="overlay overlay--scan" />
      <div className="overlay overlay--vignette" />
      <div className="stars" aria-hidden />
      <div className="stars stars--2" aria-hidden />
      <div className="stars stars--3" aria-hidden />

      <main className="content">
        <h1 className="title">
          <span>L.A.D</span>
          <small>{user ? "Learn Autonomous Driving" : "LOGIN TO L.A.D"}</small>
        </h1>

        {!user ? <LoginCard onLogin={login} loading={loading} /> : (
          <div className="menu">
            <button ref={startRef} onClick={handleStart} className="start">▶ START Learning</button>
            <button className="btn ghost" disabled>MISIONS</button>
            <button className="btn ghost" disabled>RESEARCH</button>
            <button className="btn" onClick={logout}>Logout</button>
          </div>
        )}

        <div className="hints">
          {user ? (
            <>
              <div>Developed at Universidad Adolfo Ibáñez</div>
            </>
          ) : (
            <div>PLACEHOLDER</div>
          )}
        </div>
      </main>

      <footer className="footer">© {new Date().getFullYear()} Nicomedes Pommier</footer>
    </div>
  );
}

function LoginCard({ onLogin, loading }) {
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");

  async function submit(e) {
    e.preventDefault();
    setError("");
    try {
      const ok = await onLogin(username, password);
      if (!ok) setError("Could not login");
    } catch (err) {
      setError(err.message || "Login failed");
    }
  }

  return (
    <form className="login" onSubmit={submit}>
      <div className="login__panel">
        <div className="login__title">LOGIN</div>
        <label className="login__field">
          <span>Username</span>
          <input value={username} onChange={e => setUsername(e.target.value)} placeholder="player1" required />
        </label>
        <label className="login__field">
          <span>Password</span>
          <input type="password" value={password} onChange={e => setPassword(e.target.value)} placeholder="••••••••" required />
        </label>
        {error && <div className="login__error">{error}</div>}
        <button type="submit" className="start" disabled={loading}>{loading ? "Connecting…" : "▶ START"}</button>
      </div>
    </form>
  );
}
