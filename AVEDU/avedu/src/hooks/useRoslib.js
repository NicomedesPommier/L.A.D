// src/hooks/useRoslib.js
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import ROSLIB from "roslib";
import { ROS_WS_URL } from "../config";

const isValidWsUrl = (u) => /^wss?:\/\/[^\s]+$/i.test(String(u || ""));

export function useRoslib(url = ROS_WS_URL) {
  const rosRef = useRef(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    // 1) Crear instancia sin URL y fijar listeners ANTES de intentar conectar
    const ros = new ROSLIB.Ros();
    rosRef.current = ros;

    // Sink para evitar EventEmitter2 "error" sin listener (se queda aunque quitemos otros)
    const sinkError = () => {};
    ros.on("error", sinkError);

    const handleConnection = () => setConnected(true);
    const handleClose = () => setConnected(false);
    const handleError = (e) => {
      // eslint-disable-next-line no-console
      console.error("[roslib] error:", e?.message || e);
      setConnected(false);
    };

    ros.on("connection", handleConnection);
    ros.on("close", handleClose);
    ros.on("error", handleError);

    // 2) Conectar en el próximo tick (mitiga StrictMode doble-mount)
    const t = setTimeout(() => {
      try {
        if (!isValidWsUrl(url)) throw new Error(`Invalid WebSocket URL: ${url}`);
        ros.connect(url);
      } catch (e) {
        handleError(e);
      }
    }, 0);

    return () => {
      clearTimeout(t);
      try { ros.close(); } catch (_) {}
      try {
        // 3) Limpieza dura: remover TODOS los listeners para no dejar emisores “huérfanos”
        if (typeof ros.removeAllListeners === "function") {
          ros.removeAllListeners("connection");
          ros.removeAllListeners("close");
          ros.removeAllListeners("error");
        }
      } catch (_) {}
      rosRef.current = null;
    };
  }, [url]);

  // --- APIs ESTABLES ---
  const subscribeTopic = useCallback((name, messageType, onMessage, opts = {}) => {
    const ros = rosRef.current;
    if (!ros) return () => {};

    const topic = new ROSLIB.Topic({
      ros,
      name,
      messageType,
      throttle_rate: opts.throttle_rate ?? 50,
      queue_size: opts.queue_size ?? 1,
      queue_length: opts.queue_length,
    });

    const handler = (msg) => { try { onMessage?.(msg); } catch (_) {} };
    try { topic.subscribe(handler); } catch (_) {}

    return () => {
      try { topic.unsubscribe(handler); } catch (_) {}
      try { topic.unsubscribe(); } catch (_) {}
    };
  }, []);

  const advertise = useCallback((name, messageType, opts = {}) => {
    const ros = rosRef.current;
    if (!ros) return { publish: () => {}, unadvertise: () => {} };

    const topic = new ROSLIB.Topic({
      ros,
      name,
      messageType,
      queue_size: opts.queue_size ?? 1,
    });

    try { topic.advertise(); } catch (_) {}

    return {
      publish: (msg) => { try { topic.publish(new ROSLIB.Message(msg)); } catch (_) {} },
      unadvertise: () => { try { topic.unadvertise(); } catch (_) {} },
    };
  }, []);

  const callService = useCallback((name, serviceType, request = {}) => {
    const ros = rosRef.current;
    if (!ros) return Promise.reject(new Error("ROS not connected"));

    const svc = new ROSLIB.Service({ ros, name, serviceType });
    const req = new ROSLIB.ServiceRequest(request);

    return new Promise((resolve, reject) => {
      try {
        svc.callService(req, resolve, reject);
      } catch (e) {
        reject(e);
      }
    });
  }, []);

  return useMemo(
    () => ({ ros: rosRef, connected, subscribeTopic, advertise, callService }),
    [connected, subscribeTopic, advertise, callService]
  );
}
