// src/levels/rosbasic/slides/WorkspaceAndBuildSlide.jsx
import React from "react";
import "../../styles/pages/_folder.scss";

export const meta = {
  id: "ros2-structure-and-colcon",
  title: "Estructura de un workspace ROS 2 + colcon build",
  order: 5,
  objectiveCode: "ros-slide-workspace-and-colcon",
};

const ITEMS = [
  { id: "ws", label: "ros2_ws/  (workspace)", kind: "dir", desc: `Tu carpeta de trabajo. Aquí vivirán los paquetes ROS 2 dentro de "src/". 
Puedes tener múltiples workspaces y “encadenarlos” con overlays.` },
  { id: "src", label: "└─ src/", kind: "dir", desc: `Código fuente. Dentro de "src/" colocas uno o más "packages" ROS 2.
Cada paquete es una unidad que se compila/instala por separado.` },
  { id: "pkg", label: "   └─ my_package/", kind: "dir", desc: `Un paquete ROS 2. Debe tener al menos "package.xml" y archivos de build:
- Python: setup.py, setup.cfg (o pyproject.toml)
- C++: CMakeLists.txt
Además, tu código fuente: scripts/ o src/, launch/, msg/srv/action si defines interfaces.` },
  { id: "pkgxml", label: "      ├─ package.xml", kind: "file", desc: `Manifiesto del paquete. Declara nombre, versión, mantenedores y dependencias.
colcon y ament lo usan para resolver el grafo de dependencias.` },
  { id: "setup", label: "      ├─ setup.py / setup.cfg (Python)", kind: "file", desc: `Metadatos y reglas de instalación para paquetes Python.
Permite instalar scripts como entry points y copiar recursos durante el build/instalación.` },
  { id: "cmake", label: "      ├─ CMakeLists.txt (C++)", kind: "file", desc: `Script de CMake para paquetes C++. Declara ejecutables, include dirs, dependencias
(y targets de instalación) usando ament_cmake.` },
  { id: "code", label: "      └─ src/  /  scripts/  /  launch/", kind: "dir", desc: `Tu código:
- src/: fuentes C++ o módulos Python
- scripts/: scripts Python ejecutables (con shebang y permisos)
- launch/: archivos *.launch.py para ejecutar nodos/composiciones.` },
  { id: "build", label: "build/  (autogenerado)", kind: "dir", desc: `Salida intermedia del build (objetos, CMake cache, etc.). 
colcon la recrea; puedes borrarla para “limpiar” (clean build).` },
  { id: "install", label: "install/  (autogenerado)", kind: "dir", desc: `Resultado instalable del workspace. Al "source install/setup.bash" 
tu entorno verá los ejecutables y recursos del workspace.` },
  { id: "log", label: "log/  (autogenerado)", kind: "dir", desc: `Logs de compilar/ejecutar. Útil para depurar errores de colcon o test.` },
];

const DEFAULT_CMD =
  "colcon build --symlink-install\n\n# luego de compilar:\nsource install/setup.bash";

function SectionCard({ title, children, right }) {
  return (
    <div className={`section-card ${right ? "section-card--right" : ""}`}>
      <div className="section-card__title">{title}</div>
      {children}
    </div>
  );
}

export default function WorkspaceAndBuildSlide({ onObjectiveHit }) {
  const [selected, setSelected] = React.useState("ws");

  const current = React.useMemo(
    () => ITEMS.find((i) => i.id === selected) || ITEMS[0],
    [selected]
  );

  const markDone = () => onObjectiveHit?.(meta.objectiveCode);

  return (
    <div className="slide-wrap" key={meta.id}>
      <h2>{meta.title}</h2>

      {/* Franja superior: árbol + detalle */}
      <SectionCard title="Estructura básica de un workspace">
        <div className="ws-grid">
          {/* Árbol simple, clicable */}
          <div className="tree">
            <pre className="tree__pre">
              {ITEMS.slice(0, 7).map((it) => (
                <div
                  key={it.id}
                  role="button"
                  onClick={() => setSelected(it.id)}
                  title="Click para ver detalle"
                  className={`tree__item ${selected === it.id ? "is-selected" : ""}`}
                >
                  <code>{it.label}</code>
                </div>
              ))}
              <div className="tree__spacer" />
              {ITEMS.slice(7).map((it) => (
                <div
                  key={it.id}
                  role="button"
                  onClick={() => setSelected(it.id)}
                  title="Click para ver detalle"
                  className={`tree__item ${selected === it.id ? "is-selected" : ""}`}
                >
                  <code>{it.label}</code>
                </div>
              ))}
            </pre>
          </div>

          {/* Detalle del ítem */}
          <div className="detail">
            <div className="detail__label">
              {current?.label?.replace(/^\s+/, "")}
            </div>
            <div className="detail__desc">
              {current?.desc}
            </div>
          </div>
        </div>
      </SectionCard>

      {/* Franja inferior: colcon build */}
      <SectionCard title="¿Qué hace colcon build?">
        <div className="colcon-grid">
          <ul className="colcon-list">
            <li>
              <b>Descubre paquetes</b> en <code>src/</code> (busca <code>package.xml</code>) y <b>resuelve dependencias</b>.
            </li>
            <li>
              Ejecuta el <b>build system</b> correspondiente:
              <ul className="colcon-sublist">
                <li><code>ament_python</code> → instala módulos/scripts Python.</li>
                <li><code>ament_cmake</code> → invoca CMake/Make para C++, genera e instala ejecutables y recursos.</li>
              </ul>
            </li>
            <li>
              Genera <code>build/</code> (artefactos intermedios), <code>install/</code> (resultado instalable) y <code>log/</code> (registros).
            </li>
            <li>
              Tras compilar, debes <b>activar el overlay</b>:
              <pre className="code-inline">source install/setup.bash</pre>
              así tu shell ve los ejecutables/paquetes del workspace.
            </li>
          </ul>

          <div className="cmd-card">
            <div className="cmd-card__title">Comando típico</div>
            <pre className="cmd-card__code">{DEFAULT_CMD}</pre>
            <div className="cmd-card__actions">
              <button
                className="btn"
                onClick={() => {
                  navigator.clipboard?.writeText(DEFAULT_CMD).catch(() => {});
                }}
              >
                Copiar
              </button>
              <button className="btn" onClick={markDone}>
                Marcar logrado
              </button>
            </div>
            <div className="cmd-card__hint">
              <b>--symlink-install</b>: en desarrollo Python, instala enlaces simbólicos
              para que los cambios en código se reflejen sin reinstalar.
            </div>
          </div>
        </div>
      </SectionCard>
    </div>
  );
}
