// src/components/gazebo/CameraSelector.jsx
import React, { useState } from "react";
import CameraFeedUniversal from "./CameraFeedUniversal";

const CAMERA_OPTIONS = [
  { name: "RGB Camera", topic: "/qcar/rgb/image_raw", icon: "📹", compressed: false },
  { name: "CSI Front", topic: "/qcar/csi_front/image_raw", icon: "⬆️", compressed: false },
  { name: "CSI Right", topic: "/qcar/csi_right/image_raw", icon: "➡️", compressed: false },
  { name: "CSI Back", topic: "/qcar/csi_back/image_raw", icon: "⬇️", compressed: false },
  { name: "CSI Left", topic: "/qcar/csi_left/image_raw", icon: "⬅️", compressed: false },
];

export default function CameraSelector({ connected, subscribeTopic }) {
  const [selectedCamera, setSelectedCamera] = useState(0);
  const [showAllCameras, setShowAllCameras] = useState(false);

  const currentCamera = CAMERA_OPTIONS[selectedCamera];

  if (showAllCameras) {
    return (
      <div className="camera-selector">
        <div className="camera-selector__header">
          <h3>All Cameras</h3>
          <button
            className="btn btn--sm"
            onClick={() => setShowAllCameras(false)}
            style={{ padding: ".4rem .75rem" }}
          >
            Single View
          </button>
        </div>

        <div className="camera-grid">
          {CAMERA_OPTIONS.map((cam, idx) => (
            <div key={idx} className="camera-grid__item">
              <CameraFeedUniversal
                connected={connected}
                subscribeTopic={subscribeTopic}
                topicName={cam.topic}
                title={`${cam.icon} ${cam.name}`}
                compressed={cam.compressed}
              />
            </div>
          ))}
        </div>
      </div>
    );
  }

  return (
    <div className="camera-selector">
      <div className="camera-selector__header">
        <div className="camera-selector__tabs">
          {CAMERA_OPTIONS.map((cam, idx) => (
            <button
              key={idx}
              className={`camera-selector__tab ${idx === selectedCamera ? "active" : ""}`}
              onClick={() => setSelectedCamera(idx)}
              title={cam.name}
            >
              {cam.icon} {cam.name}
            </button>
          ))}
        </div>
        <button
          className="btn btn--sm"
          onClick={() => setShowAllCameras(true)}
          style={{ padding: ".4rem .75rem", marginLeft: ".5rem" }}
        >
          Grid View
        </button>
      </div>

      <CameraFeedUniversal
        connected={connected}
        subscribeTopic={subscribeTopic}
        topicName={currentCamera.topic}
        title={`${currentCamera.icon} ${currentCamera.name}`}
        compressed={currentCamera.compressed}
      />
    </div>
  );
}
