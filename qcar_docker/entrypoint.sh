#!/usr/bin/env bash
set -e 
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# ===== Config =====
: "${STATIC_PORT:=7000}"
: "${WVS_PORT:=8080}"
: "${USER_DEFINED_CORS_FLAG:=${CORS_ALLOW_ORIGIN+x}}"
: "${CORS_ALLOW_ORIGIN:=http://localhost:8000}"
: "${ENABLE_ROSAPI:=1}"
: "${ENABLE_JSP:=1}"
: "${ENABLE_WVS:=0}"
: "${ENABLE_TURTLESIM:=0}"
# Args opcionales para xacro (ej: "use_camera:=true")
: "${XACRO_ARGS:=}"

if [ "${USER_DEFINED_CORS_FLAG}" != "x" ]; then
  CONFIG_PATH="${IP_CONFIG_PATH:-}"
  if [ -n "${CONFIG_PATH}" ] && [ -r "${CONFIG_PATH}" ]; then
    EXPOSED_IP="$(python3 - "${CONFIG_PATH}" <<'PY'
import json
import sys

path = sys.argv[1]
try:
    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)
except (OSError, json.JSONDecodeError):
    sys.exit(0)

ip = (data.get("exposed_ip") or "").strip()
if ip:
    print(ip)
PY
)"
    if [ -n "${EXPOSED_IP}" ]; then
      CORS_ALLOW_ORIGIN="http://${EXPOSED_IP}:3000"
    fi
  fi
fi

# ===== Rutas desde qcar_description instalado =====
QCAR_DESC_PREFIX="$(ros2 pkg prefix qcar_description)"
QCAR_DESC_SHARE="${QCAR_DESC_PREFIX}/share/qcar_description"
URDF_DIR="${QCAR_DESC_SHARE}/urdf"
MAIN_XACRO="${URDF_DIR}/qcar_ros2.urdf.xacro"
TMP_URDF="/tmp/robot_model/robot.urdf"

echo "[entrypoint] qcar_description: ${QCAR_DESC_SHARE}"
echo "[entrypoint] XACRO: ${MAIN_XACRO}"
[ -f "${MAIN_XACRO}" ] || { echo "[ERROR] No existe ${MAIN_XACRO}"; exit 1; }

mkdir -p /tmp/robot_model
xacro "${MAIN_XACRO}" ${XACRO_ARGS} > "${TMP_URDF}"
grep -q "<robot" "${TMP_URDF}" || { echo "[ERROR] URDF inválido"; exit 1; }

# Copia al package para servir por HTTP
cp -f "${TMP_URDF}" "${URDF_DIR}/robot_runtime.urdf"
echo "[entrypoint] URDF HTTP: http://localhost:${STATIC_PORT}/qcar_description/urdf/robot_runtime.urdf"

# Lanza todo con flags (Opción 1)
exec ros2 launch qcar_bringup web_viz.launch.py \
  cors_allow_origin:="${CORS_ALLOW_ORIGIN}" \
  static_port:="${STATIC_PORT}" \
  enable_rosapi:="$([ "${ENABLE_ROSAPI}" = "1" ] && echo true || echo false)" \
  enable_jsp:="$([ "${ENABLE_JSP}" = "1" ] && echo true || echo false)" \
