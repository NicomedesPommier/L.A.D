L.A.D (Learn Autonomous Driving) es una plataforma web creada para acompañar cursos de robótica y conducción autónoma. Este README reúne en un solo lugar cómo funciona la aplicación, qué servicios necesita alrededor y cómo personalizar los contenidos para tu laboratorio.

## Tabla de contenido

1. [Visión general](#visión-general)
2. [Cómo funciona el flujo de aprendizaje](#cómo-funciona-el-flujo-de-aprendizaje)
3. [Servicios externos requeridos](#servicios-externos-requeridos)
4. [Requisitos y variables de entorno](#requisitos-y-variables-de-entorno)
5. [Instalación y scripts de npm](#instalación-y-scripts-de-npm)
6. [Estructura del código](#estructura-del-código)
7. [Integración con ROS 2 y simuladores](#integración-con-ros-2-y-simuladores)
8. [Orquestación con Docker Compose](#orquestación-con-docker-compose)
9. [Gestión de unidades, niveles y objetivos](#gestión-de-unidades-niveles-y-objetivos)
10. [Buenas prácticas de versionado](#buenas-prácticas-de-versionado)
11. [Próximos pasos sugeridos](#próximos-pasos-sugeridos)

## Visión general

La aplicación React se centra en guiar al estudiantado por un catálogo de **unidades** y **niveles** que combinan teoría, prácticas y simulaciones. Utiliza React Router para estructurar la navegación (`/learn/:unit/:level`), la Context API para manejar autenticación y progreso, y componentes especializados para interactuar con ROS a través de rosbridge.

### Características principales

- **Acceso autenticado.** En `Home.jsx` se muestra un panel de login que solicita usuario y contraseña. Al iniciar sesión se obtiene un token JWT desde `/api/token/` y se almacena en `localStorage` mediante el `AuthContext`.
- **Catálogo progresivo.** Una vez dentro, `Learn.jsx` carga las unidades desde `/api/units/`, calcula qué niveles están completos y permite navegar entre ellos sin abandonar la página.
- **Seguimiento personalizado.** Cada nivel incluye objetivos que el backend marca como logrados. La vista mezcla los datos estáticos del catálogo con el progreso del usuario (`/api/levels/progress/me/`).
- **Widgets conectados a ROS.** En la carpeta `src/levels` se declaran misiones que utilizan hooks como `useRoslib` para publicar y suscribirse a tópicos (`REACT_APP_ROSBRIDGE_URL`). Esto permite enviar comandos, leer sensores o lanzar escenarios desde el navegador.

## Cómo funciona el flujo de aprendizaje

1. **Inicio de sesión (`Home.jsx`).**
   - El formulario llama a `login(username, password)` del `AuthContext`.
   - Si la API devuelve un token válido, la sesión se guarda y se redirige a `/learn`.
2. **Carga de catálogo (`Learn.jsx`).**
   - `Learn.jsx` solicita `GET /units/` y guarda la respuesta en estado local.
   - Si hay un `unitSlug` en la URL se selecciona esa unidad en la barra lateral; de lo contrario se muestra un placeholder.
3. **Progreso individual.**
   - Después de obtener las unidades, se pide `GET /levels/progress/me/`.
   - `mergeProgressIntoUnits` combina ambos resultados para que cada nivel indique si está completo y qué objetivos están alcanzados.
4. **Exploración de niveles (`UnitPage.jsx` y `LearnLevel.jsx`).**
   - `UnitPage.jsx` muestra una lista de niveles con indicadores de estado.
   - `LearnLevel.jsx` recupera la definición del nivel (slides, videos, widgets ROS) y permite reportar objetivos completados.

> 🧭 **Tip:** La consola del navegador registra cada petición (`[apiFetch]`) y su respuesta, útil para depurar integraciones con la API.

## Servicios externos requeridos

La aplicación asume la existencia de tres servicios externos. Puedes desplegarlos de forma local o en contenedores.

| Servicio | Rol | Endpoint por defecto |
| --- | --- | --- |
| **Backend REST** | Autenticación, catálogo de unidades/niveles y progreso del estudiante. | `http://localhost:8000/api` |
| **Base de datos** | Persistencia del backend (PostgreSQL, SQLite, etc.). | Según la configuración del backend |
| **rosbridge** | Puente WebSocket para interactuar con ROS 2. | `ws://localhost:9090` |

El backend de referencia se puede construir con Django REST Framework, FastAPI u otro framework que exponga los endpoints esperados. rosbridge debe ejecutarse sobre el workspace ROS con los paquetes de las misiones que utilizará el curso.

## Requisitos y variables de entorno

### Requisitos mínimos

- Node.js ≥ 18 y npm.
- Docker (opcional pero recomendado) para levantar backend y rosbridge en contenedores reproducibles.
- Acceso a un backend que implemente los endpoints `/api/token/`, `/api/units/` y `/api/levels/progress/me/`.

### Variables de entorno

Define estas variables antes de compilar o ejecutar la app (puedes usar un archivo `.env` en la raíz del proyecto):

| Variable | Descripción | Valor por defecto |
| --- | --- | --- |
| `REACT_APP_API_BASE` | URL base para las peticiones REST. | `http://localhost:8000/api` |
| `REACT_APP_ROSBRIDGE_URL` | URL WebSocket hacia rosbridge. | `ws://localhost:9090` |

> ⚠️ Las variables se inyectan en tiempo de build. Si cambias la URL después de `npm run build`, recompila o reconstruye la imagen Docker.

## Instalación y scripts de npm

```bash
npm install        # Instala dependencias
npm start          # Servidor de desarrollo en http://localhost:3000
npm test           # Ejecuta pruebas de React (Jest + Testing Library)
npm run build      # Compila la versión de producción en /build
npm run lint       # Si añades ESLint, puedes exponerlo aquí
```

- En modo desarrollo se utiliza `react-scripts` con recarga en caliente.
- `npm test` ejecuta las pruebas incluidas (por defecto `App.test.js`).
- `npm run build` genera archivos estáticos listos para un servidor como Nginx o para empaquetar en Docker.

## Estructura del código

```
src/
├── App.jsx / App.js       # Rutas principales de React Router
├── components/            # UI reutilizable y widgets de simulación
├── context/               # Contextos de autenticación y progreso
├── hooks/                 # Hooks para ROS, peticiones y utilidades
├── levels/                # Definiciones de las misiones por nivel
├── pages/                 # Pantallas de Home, Learn, UnitPage y Level
├── parches/               # Fixes o overrides puntuales
├── styles/                # Estilos SCSS organizados por vistas y componentes
└── config.js              # Punto centralizado para URLs y toggles
```

Algunos archivos clave para entender el flujo:

- `src/context/AuthContext.jsx`: maneja login/logout, guarda el token en `localStorage` y expone `apiFetch` con cabeceras autenticadas.
- `src/pages/Learn.jsx`: descarga catálogo y progreso, controla el estado de la barra lateral y decide qué unidad está activa.
- `src/pages/LearnLevel.jsx`: renderiza el contenido del nivel seleccionado y reporta avances.
- `src/hooks/useRoslib.js`: encapsula la conexión a rosbridge (suscripciones, publicaciones, acciones).

## Integración con ROS 2 y simuladores

Las misiones aprovechan rosbridge para comunicarse con ROS 2. Desde el frontend se pueden realizar acciones como:

- Publicar en tópicos (`/cmd_vel`, `/mission/goal`).
- Suscribirse a sensores para actualizar widgets en tiempo real.
- Invocar servicios o acciones (por ejemplo, iniciar un escenario Gazebo o validar un checkpoint).

Un contenedor base puede iniciarse con:

```bash
docker run --rm -it \
  -p 9090:9090 \
  -v $(pwd)/ros2_ws:/root/ros2_ws \
  osrf/ros:humble-desktop \
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Desde los niveles puedes parametrizar a qué tópicos o servicios conectarte. Mantén la misma convención en el backend para que los objetivos se marquen como completados cuando el simulador informe los resultados.

## Orquestación con Docker Compose

```yaml
services:
  db:
    image: postgres:15
    environment:
      - POSTGRES_DB=lad
      - POSTGRES_USER=lad_user
      - POSTGRES_PASSWORD=lad_pass
    volumes:
      - lad_pgdata:/var/lib/postgresql/data

  api:
    image: lad/api:latest
    ports:
      - "8000:8000"
    environment:
      - DJANGO_SECRET_KEY=changeme
      - DATABASE_URL=postgres://lad_user:lad_pass@db:5432/lad
    depends_on:
      - db

  rosbridge:
    image: osrf/ros:humble-desktop
    command: ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    ports:
      - "9090:9090"

  frontend:
    build: .
    ports:
      - "3000:3000"
    environment:
      - REACT_APP_API_BASE=http://api:8000/api
      - REACT_APP_ROSBRIDGE_URL=ws://rosbridge:9090
    depends_on:
      - api
      - rosbridge

volumes:
  lad_pgdata:
    driver: local
```

