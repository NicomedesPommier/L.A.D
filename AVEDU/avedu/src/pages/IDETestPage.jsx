// =============================================================
// FILE: src/pages/IDETestPage.jsx
// Test page for the ROS Visual IDE components (BlockCanvas + FileExplorer)
// =============================================================
import React, { useState, useCallback, useMemo, useEffect } from "react";
import { ReactFlowProvider } from "@xyflow/react";
import { useNavigate } from "react-router-dom";
import { BlockCanvas } from "../components/ide/BlockCanvas";
import { FileExplorer } from "../components/ide/FileExplorer";
import { TabBar } from "../components/ide/TabBar";
import { Terminal } from "../components/ide/Terminal";
import { URDFViewer } from "../components/ide/URDFViewer";
import { CategorizedPalette } from "../components/blocks";
import { paletteCategorized } from "../components/blocks";
import { computeUrdfXml } from "../components/blocks/urdf-helpers";
import CanvasSelector from "../components/ide/CanvasSelector";
import ThemeToggle from "../components/ThemeToggle";
import fileApi from "../services/fileApi";
import "../styles/_rosflow.scss";
import "../styles/pages/_ide-test.scss";
import "../styles/components/_canvas-selector.scss";

function IDETestPageInner() {
  const navigate = useNavigate();

  // Canvas state
  const [canvas, setCanvas] = useState(null);
  const [canvasLoading, setCanvasLoading] = useState(false);
  const [showCanvasSelector, setShowCanvasSelector] = useState(true);

  // File tree state
  const [fileTree, setFileTree] = useState([
    {
      name: "src",
      path: "/src",
      type: "directory",
      children: [
        {
          name: "my_robot",
          path: "/src/my_robot",
          type: "directory",
          children: [
            {
              name: "urdf",
              path: "/src/my_robot/urdf",
              type: "directory",
              children: [
                {
                  name: "robot.urdf",
                  path: "/src/my_robot/urdf/robot.urdf",
                  type: "file",
                  unsaved: false,
                },
              ],
            },
            {
              name: "package.xml",
              path: "/src/my_robot/package.xml",
              type: "file",
            },
            {
              name: "setup.py",
              path: "/src/my_robot/setup.py",
              type: "file",
            },
          ],
        },
      ],
    },
    {
      name: "build",
      path: "/build",
      type: "directory",
      children: [],
    },
    {
      name: "install",
      path: "/install",
      type: "directory",
      children: [],
    },
  ]);

  // Current file and graph state
  const [currentFile, setCurrentFile] = useState("/src/my_robot/urdf/robot.urdf");
  const [graphs, setGraphs] = useState({
    "/src/my_robot/urdf/robot.urdf": {
      nodes: [],
      edges: [],
    },
  });
  const [generatedCode, setGeneratedCode] = useState("");

  // Tabs state
  const [openTabs, setOpenTabs] = useState([
    {
      path: "/src/my_robot/urdf/robot.urdf",
      name: "robot.urdf",
      type: "file",
      unsaved: false,
    },
  ]);
  const [activeTab, setActiveTab] = useState("/src/my_robot/urdf/robot.urdf");
  const [showTerminal, setShowTerminal] = useState(false);
  const [fileExplorerCollapsed, setFileExplorerCollapsed] = useState(false);

  // Handle canvas selection from CanvasSelector
  const handleCanvasSelect = useCallback(async (selectedCanvas) => {
    try {
      setCanvasLoading(true);
      setCanvas(selectedCanvas);

      // Load file tree
      const tree = await fileApi.getFileTree(selectedCanvas.id);
      if (tree.length > 0) {
        setFileTree(tree);
      }

      // Hide canvas selector and show IDE
      setShowCanvasSelector(false);
    } catch (error) {
      console.error("Canvas load error:", error);
    } finally {
      setCanvasLoading(false);
    }
  }, []);

  // Get current graph
  const currentGraph = useMemo(() => {
    return graphs[currentFile] || { nodes: [], edges: [] };
  }, [graphs, currentFile]);

  // Handle file selection
  const handleFileSelect = useCallback((path) => {
    setCurrentFile(path);
    setActiveTab(path);

    // Add to tabs if not already open
    setOpenTabs((prev) => {
      if (prev.some((tab) => tab.path === path)) return prev;

      const fileName = path.split("/").pop();
      return [
        ...prev,
        {
          path,
          name: fileName,
          type: "file",
          unsaved: false,
        },
      ];
    });
  }, []);

  // Handle graph changes
  const handleGraphChange = useCallback(
    ({ nodes, edges }) => {
      setGraphs((prev) => ({
        ...prev,
        [currentFile]: { nodes, edges },
      }));
    },
    [currentFile]
  );

  // Handle code generation
  const handleCodeGenerated = useCallback((result) => {
    // Handle both object {xml, robotId} and string returns
    const code = typeof result === 'string' ? result : (result?.xml || "");
    setGeneratedCode(code);
  }, []);

  // File operations
  const handleFileCreate = useCallback(
    async (path, type) => {
      if (!canvas) return;

      try {
        await fileApi.createFile(canvas.id, {
          path,
          file_type: type,
          content: type === "file" ? "" : undefined,
        });

        // Reload file tree
        const tree = await fileApi.getFileTree(canvas.id);
        setFileTree(tree);
      } catch (error) {
        console.error(`Failed to create ${type}:`, error);
      }
    },
    [canvas]
  );

  const handleFileDelete = useCallback(
    async (path) => {
      if (!canvas) return;

      if (!window.confirm(`Delete ${path}?`)) return;

      try {
        // Find file by path
        const files = await fileApi.listFiles(canvas.id);
        const file = files.find((f) => f.path === path);

        if (file) {
          await fileApi.deleteFile(canvas.id, file.id);

          // Reload file tree
          const tree = await fileApi.getFileTree(canvas.id);
          setFileTree(tree);
        }
      } catch (error) {
        console.error("Failed to delete:", error);
      }
    },
    [canvas]
  );

  const handleFileRename = useCallback(
    async (oldPath, newPath) => {
      if (!canvas) return;

      try {
        // Find file by old path
        const files = await fileApi.listFiles(canvas.id);
        const file = files.find((f) => f.path === oldPath);

        if (file) {
          await fileApi.updateFile(canvas.id, file.id, { path: newPath });

          // Reload file tree
          const tree = await fileApi.getFileTree(canvas.id);
          setFileTree(tree);
        }
      } catch (error) {
        console.error("Failed to rename:", error);
      }
    },
    [canvas]
  );

  // Save current file
  const handleSave = useCallback(async () => {
    if (!canvas || !generatedCode || !currentFile) {
      return;
    }

    try {
      await fileApi.saveGeneratedCode(canvas.id, currentFile, generatedCode);

      // Reload file tree to reflect changes
      const tree = await fileApi.getFileTree(canvas.id);
      setFileTree(tree);
    } catch (error) {
      console.error("Failed to save:", error);
    }
  }, [canvas, currentFile, generatedCode]);

  // Clear canvas
  const handleClear = useCallback(() => {
    if (window.confirm("Clear the canvas?")) {
      setGraphs((prev) => ({
        ...prev,
        [currentFile]: { nodes: [], edges: [] },
      }));
      setGeneratedCode("");
    }
  }, [currentFile]);

  // Tab management
  const handleTabSelect = useCallback((path) => {
    if (path === "terminal") {
      setShowTerminal(true);
      setActiveTab("terminal");
    } else {
      setShowTerminal(false);
      setActiveTab(path);
      setCurrentFile(path);
    }
  }, []);

  const handleTabClose = useCallback((path) => {
    setOpenTabs((prev) => {
      const filtered = prev.filter((tab) => tab.path !== path);

      // If closing active tab, switch to another tab
      if (activeTab === path && filtered.length > 0) {
        const nextTab = filtered[filtered.length - 1];
        setActiveTab(nextTab.path);
        setCurrentFile(nextTab.path);
        setShowTerminal(false);
      } else if (filtered.length === 0) {
        setActiveTab(null);
        setCurrentFile(null);
        setShowTerminal(false);
      }

      return filtered;
    });
  }, [activeTab]);

  const handleToggleTerminal = useCallback(() => {
    if (showTerminal) {
      setShowTerminal(false);
      if (openTabs.length > 0) {
        const lastFileTab = openTabs[openTabs.length - 1];
        setActiveTab(lastFileTab.path);
        setCurrentFile(lastFileTab.path);
      }
    } else {
      setShowTerminal(true);
      setActiveTab("terminal");

      // Add terminal tab if not already present
      setOpenTabs((prev) => {
        if (prev.some((tab) => tab.path === "terminal")) return prev;
        return [...prev, { path: "terminal", name: "Terminal", type: "terminal", unsaved: false }];
      });
    }
  }, [showTerminal, openTabs]);

  const handleCommandExecute = useCallback(async (command, callback) => {
    if (!canvas) {
      callback?.("Error: Canvas not loaded");
      return;
    }

    try {
      const result = await fileApi.executeCommand(canvas.id, command);

      // Send output to terminal
      if (result.output) {
        callback?.(result.output);
      }
      if (result.error) {
        callback?.(`\x1b[31m${result.error}\x1b[0m`); // Red color for errors
      }
    } catch (error) {
      callback?.(`Error: ${error.message}`);
    }
  }, [canvas]);

  // Show canvas selector if no canvas is selected
  if (showCanvasSelector) {
    return <CanvasSelector onCanvasSelect={handleCanvasSelect} />;
  }

  // Show loading state
  if (canvasLoading) {
    return (
      <div className="ide-test">
        <div className="ide-test__loading">Loading workspace...</div>
      </div>
    );
  }

  return (
    <div className="ide-test">
      {/* Header */}
      <header className="ide-test__header">
        <button className="ide-test__back" onClick={() => setShowCanvasSelector(true)}>
          ← Change Workspace
        </button>
        <h1 className="ide-test__title">
          <span className="ide-test__title-main">ROS Visual IDE</span>
          <span className="ide-test__title-sub">{canvas?.name || "Test Environment"}</span>
        </h1>
        <div className="ide-test__actions">
          <button
            className="btn btn--small"
            onClick={handleToggleTerminal}
            title="Toggle Terminal"
          >
            📟 Terminal
          </button>
          <button className="btn btn--small" onClick={handleClear}>
            Clear
          </button>
          <button className="btn btn--small btn--primary" onClick={handleSave}>
            💾 Save
          </button>
          <ThemeToggle />
        </div>
      </header>

      {/* Main Layout */}
      <div className={`ide-test__layout ${fileExplorerCollapsed ? 'ide-test__layout--collapsed' : ''}`}>
        {/* Left Sidebar - File Explorer */}
        <aside className={`ide-test__explorer ${fileExplorerCollapsed ? 'ide-test__explorer--collapsed' : ''}`}>
          <div className="ide-test__explorer-header">
            <span className="ide-test__explorer-title">FILES</span>
            <button
              className="ide-test__explorer-toggle"
              onClick={() => setFileExplorerCollapsed(!fileExplorerCollapsed)}
              title={fileExplorerCollapsed ? "Expand file explorer" : "Collapse file explorer"}
            >
              {fileExplorerCollapsed ? '▶' : '◀'}
            </button>
          </div>
          {!fileExplorerCollapsed && (
            <FileExplorer
              files={fileTree}
              currentFile={currentFile}
              onFileSelect={handleFileSelect}
              onFileCreate={handleFileCreate}
              onFileDelete={handleFileDelete}
              onFileRename={handleFileRename}
              loading={canvasLoading}
            />
          )}
        </aside>

        {/* Center - Canvas with Palette on Top */}
        <main className="ide-test__canvas">
          {/* Block Palette - Top Tabs */}
          <div className="ide-test__palette">
            <CategorizedPalette
              categories={paletteCategorized}
              defaultCategory="URDF"
            />
          </div>

          {/* Tab Bar */}
          <TabBar
            tabs={openTabs}
            activeTab={activeTab}
            onTabSelect={handleTabSelect}
            onTabClose={handleTabClose}
          />

          {/* Content Area */}
          <div className="ide-test__canvas-content">
            {showTerminal ? (
              <Terminal
                onCommandExecute={handleCommandExecute}
                workingDirectory="/workspaces"
                username="developer"
                canvasId="test-workspace"
              />
            ) : currentFile ? (
              <BlockCanvas
                key={currentFile}
                initialNodes={currentGraph.nodes}
                initialEdges={currentGraph.edges}
                onGraphChange={handleGraphChange}
                codeGenerator={computeUrdfXml}
                onCodeGenerated={handleCodeGenerated}
                readOnly={false}
                canvasId={canvas?.id}
              />
            ) : (
              <div className="ide-test__empty">
                <h3>No file selected</h3>
                <p>Select a file from the explorer or create a new one</p>
              </div>
            )}
          </div>
        </main>

        {/* Right Sidebar - URDF Viewer & Code Preview */}
        <aside className="ide-test__sidebar">
          {/* URDF Viewer */}
          <div className="ide-test__urdf-viewer">
            <URDFViewer xmlCode={generatedCode} />
          </div>

          {/* Generated Code */}
          <div className="ide-test__code">
            <div className="ide-test__code-header">
              <span className="ide-test__code-title">Generated Code</span>
              <button
                className="ide-test__code-copy"
                onClick={() => {
                  navigator.clipboard.writeText(generatedCode);
                }}
                disabled={!generatedCode}
              >
                Copy
              </button>
            </div>
            <pre className="ide-test__code-content">
              {generatedCode || "// No code generated yet\n// Drag blocks to canvas to generate code"}
            </pre>
          </div>
        </aside>
      </div>

      {/* Info Panel */}
      <div className="ide-test__info">
        <div className="ide-test__info-item">
          <strong>📦 Components:</strong> BlockCanvas, FileExplorer, CategorizedPalette
        </div>
        <div className="ide-test__info-item">
          <strong>🎯 Features:</strong> Drag blocks, Connect nodes, Generate code, Manage files
        </div>
        <div className="ide-test__info-item">
          <strong>⚠️ Status:</strong> Test mode - Backend integration pending
        </div>
      </div>
    </div>
  );
}

export default function IDETestPage() {
  return (
    <ReactFlowProvider>
      <IDETestPageInner />
    </ReactFlowProvider>
  );
}
