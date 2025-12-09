import React, { useState, useContext } from "react";
import { Position } from "@xyflow/react";
import HandleWithLabel from "./HandleWithLabel";
import { MeshLibrary } from "./MeshLibrary";
import fileApi from "../../services/fileApi";

/**
 * GeometryNode - Reusable block for geometry configuration
 * Supports mesh (with upload/URL/library), box, cylinder, and sphere geometries
 */
export default function GeometryNode({ id, data }) {
  const d = data || {};
  const edit = (patch) => d.onChange?.(id, patch);

  const geometry = d.geometry || { type: "box", size: [1, 1, 1] };
  const canvasId = d.canvasId; // Canvas ID from context or data

  const [meshSource, setMeshSource] = useState("manual"); // manual, upload, url, library
  const [uploadProgress, setUploadProgress] = useState(null);
  const [showLibrary, setShowLibrary] = useState(false);
  const [urlInput, setUrlInput] = useState("");

  const setGeometry = (patch) => {
    edit({ geometry: { ...geometry, ...patch } });
  };

  const setSize = (index, value) => {
    const size = [...(geometry.size || [1, 1, 1])];
    size[index] = parseFloat(value) || 1;
    setGeometry({ size });
  };

  const setScale = (index, value) => {
    const scale = [...(geometry.scale || [1, 1, 1])];
    scale[index] = parseFloat(value) || 1;
    setGeometry({ scale });
  };

  // Handle file upload
  const handleFileUpload = async (event) => {
    const file = event.target.files?.[0];
    if (!file || !canvasId) return;

    // Validate file type
    const validExtensions = [".stl", ".dae", ".obj", ".STL", ".DAE", ".OBJ"];
    const extension = file.name.substring(file.name.lastIndexOf("."));
    if (!validExtensions.includes(extension)) {
      alert("Please upload a valid mesh file (.stl, .dae, .obj)");
      return;
    }

    try {
      setUploadProgress("Uploading...");
      const result = await fileApi.uploadMesh(canvasId, file);
      setGeometry({ filename: result.file_path });
      setUploadProgress("Uploaded!");
      setTimeout(() => setUploadProgress(null), 2000);
    } catch (error) {
      console.error("Upload failed:", error);
      const errorMsg = error.message || "Upload failed";
      setUploadProgress(`Error: ${errorMsg}`);
      alert(`Upload failed: ${errorMsg}\n\nThe mesh upload API endpoint may not be implemented yet. Please check the Django backend.`);
      setTimeout(() => setUploadProgress(null), 5000);
    }
  };

  // Handle URL import
  const handleUrlImport = async () => {
    if (!urlInput || !canvasId) return;

    try {
      setUploadProgress("Importing...");
      const result = await fileApi.importMeshFromUrl(canvasId, urlInput);
      setGeometry({ filename: result.file_path });
      setUrlInput("");
      setUploadProgress("Imported!");
      setTimeout(() => setUploadProgress(null), 2000);
    } catch (error) {
      console.error("Import failed:", error);
      const errorMsg = error.message || "Import failed";
      setUploadProgress(`Error: ${errorMsg}`);
      alert(`Import failed: ${errorMsg}\n\nThe mesh import API endpoint may not be implemented yet. Please check the Django backend.`);
      setTimeout(() => setUploadProgress(null), 5000);
    }
  };

  // Handle mesh selection from library
  const handleLibrarySelect = (mesh) => {
    setGeometry({ filename: mesh.file_path });
  };

  return (
    <div className="rf-card" style={{ minWidth: 320 }}>
      <div className="rf-card__title">Geometry</div>

      <div className="rf-card__body" style={{ display: "grid", gap: ".5rem" }}>
        <div className="rf-field">
          <label>Type</label>
          <select
            className="rf-input"
            value={geometry.type || "box"}
            onChange={(e) => {
              const type = e.target.value;
              const newGeom = { type };

              if (type === "mesh") {
                newGeom.filename = geometry.filename || "";
                newGeom.scale = geometry.scale || [1, 1, 1];
              } else if (type === "box") {
                newGeom.size = geometry.size || [1, 1, 1];
              } else if (type === "cylinder") {
                newGeom.radius = geometry.radius || 0.5;
                newGeom.length = geometry.length || 1;
              } else if (type === "sphere") {
                newGeom.radius = geometry.radius || 0.5;
              }

              edit({ geometry: newGeom });
            }}
          >
            <option value="mesh">Mesh</option>
            <option value="box">Box</option>
            <option value="cylinder">Cylinder</option>
            <option value="sphere">Sphere</option>
          </select>
        </div>

        {geometry.type === "mesh" && (
          <>
            {/* Mesh Source Selection */}
            <div className="rf-field">
              <label>Mesh Source</label>
              <div className="rf-btn-group">
                <button
                  className={`rf-btn-tab ${meshSource === "manual" ? "active" : ""}`}
                  onClick={() => setMeshSource("manual")}
                >
                  Manual
                </button>
                <button
                  className={`rf-btn-tab ${meshSource === "upload" ? "active" : ""}`}
                  onClick={() => setMeshSource("upload")}
                >
                  Upload
                </button>
                <button
                  className={`rf-btn-tab ${meshSource === "url" ? "active" : ""}`}
                  onClick={() => setMeshSource("url")}
                >
                  URL
                </button>
                <button
                  className={`rf-btn-tab ${meshSource === "library" ? "active" : ""}`}
                  onClick={() => {
                    setMeshSource("library");
                    setShowLibrary(true);
                  }}
                >
                  Library
                </button>
              </div>
            </div>

            {/* Manual Path Input */}
            {meshSource === "manual" && (
              <div className="rf-field">
                <label>Mesh File Path</label>
                <input
                  className="rf-input"
                  placeholder="package://path/to/mesh.stl"
                  value={geometry.filename || ""}
                  onChange={(e) => setGeometry({ filename: e.target.value })}
                />
                <small style={{ opacity: 0.7, fontSize: "0.85em", marginTop: "0.25rem", display: "block" }}>
                  e.g., package://my_robot/meshes/body.stl
                </small>
              </div>
            )}

            {/* File Upload */}
            {meshSource === "upload" && (
              <div className="rf-field">
                <label>Upload Mesh File</label>
                <input
                  type="file"
                  className="rf-input"
                  accept=".stl,.dae,.obj,.STL,.DAE,.OBJ"
                  onChange={handleFileUpload}
                  disabled={!canvasId}
                />
                {uploadProgress && (
                  <div className="rf-upload-status">{uploadProgress}</div>
                )}
                {!canvasId && (
                  <small style={{ color: "orange", fontSize: "0.85em" }}>
                    Canvas required for upload
                  </small>
                )}
                <small style={{ opacity: 0.7, fontSize: "0.85em", marginTop: "0.25rem", display: "block" }}>
                  Supported: .stl, .dae, .obj
                </small>
              </div>
            )}

            {/* URL Import */}
            {meshSource === "url" && (
              <div className="rf-field">
                <label>Import from URL</label>
                <div style={{ display: "flex", gap: "0.5rem" }}>
                  <input
                    className="rf-input"
                    placeholder="http://example.com/mesh.stl"
                    value={urlInput}
                    onChange={(e) => setUrlInput(e.target.value)}
                    disabled={!canvasId}
                  />
                  <button
                    className="rf-btn rf-btn--primary"
                    onClick={handleUrlImport}
                    disabled={!canvasId || !urlInput}
                  >
                    Import
                  </button>
                </div>
                {uploadProgress && (
                  <div className="rf-upload-status">{uploadProgress}</div>
                )}
                {!canvasId && (
                  <small style={{ color: "orange", fontSize: "0.85em" }}>
                    Canvas required for import
                  </small>
                )}
                <small style={{ opacity: 0.7, fontSize: "0.85em", marginTop: "0.25rem", display: "block" }}>
                  e.g., http://localhost:7000/qcar_description/meshes/QCarBody.stl
                </small>
              </div>
            )}

            {/* Current Mesh Path Display */}
            {geometry.filename && (
              <div className="rf-field">
                <label>Current Mesh</label>
                <div className="rf-mesh-display">
                  <code>{geometry.filename}</code>
                </div>
              </div>
            )}

            {/* Scale */}
            <div className="rf-field">
              <label>Scale (x y z)</label>
              <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr", gap: ".3rem" }}>
                {[0, 1, 2].map((i) => (
                  <input
                    key={`scale-${i}`}
                    className="rf-input"
                    type="number"
                    step="0.1"
                    placeholder={["x", "y", "z"][i]}
                    value={geometry.scale?.[i] ?? 1}
                    onChange={(e) => setScale(i, e.target.value)}
                  />
                ))}
              </div>
            </div>
          </>
        )}

        {geometry.type === "box" && (
          <div className="rf-field">
            <label>Size (x y z)</label>
            <div className="rf-grid" style={{ gridTemplateColumns: "1fr 1fr 1fr", gap: ".3rem" }}>
              {[0, 1, 2].map((i) => (
                <input
                  key={`size-${i}`}
                  className="rf-input"
                  type="number"
                  step="0.1"
                  placeholder={["width", "depth", "height"][i]}
                  value={geometry.size?.[i] ?? 1}
                  onChange={(e) => setSize(i, e.target.value)}
                />
              ))}
            </div>
          </div>
        )}

        {(geometry.type === "cylinder" || geometry.type === "sphere") && (
          <div className="rf-field">
            <label>Radius</label>
            <input
              className="rf-input"
              type="number"
              step="0.1"
              placeholder="0.5"
              value={geometry.radius ?? 0.5}
              onChange={(e) => setGeometry({ radius: parseFloat(e.target.value) || 0.5 })}
            />
          </div>
        )}

        {geometry.type === "cylinder" && (
          <div className="rf-field">
            <label>Length</label>
            <input
              className="rf-input"
              type="number"
              step="0.1"
              placeholder="1.0"
              value={geometry.length ?? 1}
              onChange={(e) => setGeometry({ length: parseFloat(e.target.value) || 1 })}
            />
          </div>
        )}
      </div>

      <HandleWithLabel
        type="source"
        position={Position.Right}
        id="geometry"
        label="geometry"
        color="blue"
      />

      {/* Mesh Library Modal */}
      {showLibrary && canvasId && (
        <MeshLibrary
          canvasId={canvasId}
          onSelect={handleLibrarySelect}
          onClose={() => setShowLibrary(false)}
        />
      )}
    </div>
  );
}
