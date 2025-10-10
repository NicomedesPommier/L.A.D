
Este repositorio contiene un entorno Docker listo para ejecutar la simulación y los nodos de visualización web del proyecto **QCar** sobre ROS 2 Humble. La imagen construye automáticamente los paquetes `qcar_description` y `qcar_bringup`, genera el URDF en tiempo de ejecución y expone los servicios necesarios para control y visualización remota.

## Estructura del repositorio

- `Dockerfile`: define la imagen base (`osrf/ros:humble-desktop`) y compila los workspaces de ROS 2.
- `docker-compose.yml`: orquesta el contenedor con los puertos y variables de entorno más comunes.
- `entrypoint.sh`: script que genera el URDF desde Xacro y lanza `qcar_bringup/web_viz.launch.py` con parámetros configurables.
- `http_cors.py`: pequeño servidor HTTP con CORS utilizado por el launch.
- `qcar/rosws/src/`: paquetes ROS 2 del QCar (`qcar_description`, `qcar_bringup`).

## Requisitos

- Docker 20.10 o superior.
- Docker Compose v2 (o `docker compose` integrado en Docker Desktop).

## Construcción de la imagen

Clona el repositorio y, desde la carpeta raíz, ejecuta:

```bash
docker compose build
```

La construcción clona el paquete externo `tf2_web_republisher_py`, copia los paquetes locales del QCar e instala las dependencias en `/ros2_ws`.

## Ejecución

Inicia el contenedor con:

```bash
docker compose up
```

El servicio expone, por defecto, los siguientes puertos en la máquina anfitriona:

- `9090`: rosbridge para conexiones WebSocket.
- `7000`: servidor estático con el URDF generado y recursos del robot.
- `8080`: web-video-server (solo si se habilita `ENABLE_WVS`).

La salida del contenedor mostrará la ruta del URDF generado, por ejemplo `http://localhost:7000/qcar_description/urdf/robot_runtime.urdf`.

`IP_CONFIG_PATH`: ruta al archivo JSON compartido con la IP expuesta (por defecto `/config/ip_config.json`, montado desde `config/ip_config.json`).
- `CORS_ALLOW_ORIGIN`: origen permitido para peticiones WebSocket/HTTP (por defecto `http://localhost:3000`). Si no defines esta variable, el `entrypoint` leerá `IP_CONFIG_PATH` y generará automáticamente `http://<ip-configurada>:3000`.
## Variables de entorno

Puedes ajustar el comportamiento del launch editando las variables en `docker-compose.yml` o sobrescribiéndolas al ejecutar `docker compose`:

- `CORS_ALLOW_ORIGIN`: origen permitido para peticiones WebSocket/HTTP (por defecto `http://localhost:3000`).
- `ENABLE_JSP`: activa el `joint_state_publisher` (1 o 0).
- `ENABLE_ROSAPI`: activa el servicio `rosapi` (1 o 0).
- `ENABLE_WVS`: habilita `web_video_server` y el puerto 8080 (1 o 0).
- `ENABLE_TURTLESIM`: lanza `turtlesim` para pruebas (1 o 0).
- `XACRO_ARGS`: parámetros adicionales a pasar al archivo `qcar_ros2.urdf.xacro` (ej. `use_camera:=true`).
- `STATIC_PORT`, `WVS_PORT`: permiten reasignar los puertos 7000 y 8080 dentro del contenedor.

## Desarrollo y personalización

1. Modifica los paquetes `qcar_description` y `qcar_bringup` dentro de `qcar/rosws/src/`.
2. Vuelve a construir la imagen (`docker compose build`) para recompilar el workspace.
3. Ejecuta `docker compose up` para probar los cambios.

El `entrypoint.sh` vuelve a generar el URDF en cada arranque, por lo que cualquier modificación en los archivos Xacro queda reflejada inmediatamente sin reconstruir la imagen.

## Limpieza

Para detener y eliminar el contenedor creado por Compose:

```bash
docker compose down
```

Si deseas liberar la imagen local:

```bash
docker image rm qcar_docker-ros
```

---

Para dudas adicionales sobre ROS 2 o la simulación del QCar consulta la documentación interna del proyecto.