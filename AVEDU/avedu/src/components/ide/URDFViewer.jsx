// =============================================================
// FILE: src/components/ide/URDFViewer.jsx
// URDF 3D Viewer component for IDE sidebar
// =============================================================
import React, { useEffect, useRef, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid } from "@react-three/drei";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import { API_BASE } from "../../config";
import "../../styles/components/_urdf-viewer.scss";

/**
 * URDF Viewer - 3D visualization of URDF robot description for IDE sidebar
 * Displays the generated URDF XML code as a 3D model
 */
export function URDFViewer({ xmlCode }) {
  const [error, setError] = useState(null);

  return (
    <div className="urdf-viewer">
      <div className="urdf-viewer__header">
        <span className="urdf-viewer__title">URDF Viewer</span>
      </div>
      <div className="urdf-viewer__canvas">
        {error ? (
          <div className="urdf-viewer__message urdf-viewer__message--error">
            {error}
          </div>
        ) : xmlCode && xmlCode.trim() !== "" ? (
          <UrdfCanvas xmlCode={xmlCode} onError={setError} />
        ) : (
          <div className="urdf-viewer__message">
            <div className="urdf-viewer__icon">🤖</div>
            <div>Add blocks to canvas to see robot preview</div>
          </div>
        )}
      </div>
    </div>
  );
}

function UrdfCanvas({ xmlCode, onError }) {
  return (
    <Canvas
      style={{ width: "100%", height: "100%" }}
      camera={{ fov: 50, near: 0.0005, far: 5000, position: [2, 2, 2] }}
      gl={{ antialias: true, logarithmicDepthBuffer: true }}
    >
      <ambientLight intensity={0.65} />
      <directionalLight position={[5, 6, 8]} intensity={0.9} />
      {/* Z-up: floor in XY => rotate grid +90° in X */}
      <Grid
        args={[20, 40]}
        rotation={[Math.PI / 2, 0, 0]}
        position={[0, 0, 0]}
        cellColor="#00f6ff"
        sectionColor="#00c3ff"
        fadeDistance={30}
        fadeStrength={1}
      />
      <RobotFromXml xmlCode={xmlCode} onError={onError} />
      <OrbitControls
        makeDefault
        enableDamping
        dampingFactor={0.08}
        minDistance={0.01}
        maxDistance={2000}
      />
    </Canvas>
  );
}

function RobotFromXml({ xmlCode, onError }) {
  const [obj, setObj] = useState(null);
  const loaderRef = useRef(null);

  useEffect(() => {
    if (!xmlCode) return;

    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    loaderRef.current = loader;

    // Configure package path resolution
    // Convert package://workspace_{canvas_id}/meshes/{filename}
    // to {API_BASE}/workspace/workspace_{canvas_id}/meshes/{filename}
    loader.packages = (packageName) => {
      // packageName will be like "workspace_a589995b-ae6b-4475-a228-34df4e2cd57a"
      return `${API_BASE}/workspace/${packageName}`;
    };

    try {
      // Parse URDF XML string directly
      const urdf = loader.parse(xmlCode);

      if (urdf) {
        urdf.scale.set(1, 1, 1);
        urdf.frustumCulled = false;
        urdf.traverse((o) => (o.frustumCulled = false));
        setObj(urdf);
      }
    } catch (err) {
      console.error("URDF parse error:", err);
      onError?.("Failed to parse URDF");
    }

    return () => {
      loaderRef.current = null;
      setObj(null);
    };
  }, [xmlCode, onError]);

  return obj ? <primitive object={obj} /> : null;
}
