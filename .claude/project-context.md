# L.A.D Platform - Project Context

## Overview
L.A.D (Learn Autonomous Driving) is a full-stack educational platform for teaching robotics and autonomous driving through interactive web-based simulations using ROS 2.

**Target Users**: University students and educators in robotics/autonomous driving courses
**Core Value**: No local installation required - students access real ROS 2 simulations via browser

## Architecture

### High-Level System Design
```
┌─────────────────────────────────────────────────────────┐
│                     STUDENT BROWSER                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐ │
│  │ React UI     │  │ JWT Auth     │  │ ROS Widgets  │ │
│  │ (Port 3000)  │  │ Context      │  │ (ROSLIB.js)  │ │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘ │
└─────────┼──────────────────┼──────────────────┼─────────┘
          │ HTTP             │ HTTP              │ WebSocket
          ▼                  ▼                   ▼
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│ React Frontend  │  │ Django Backend  │  │  ROS 2 Docker   │
│  (AVEDU)        │  │  (LAD)          │  │  (qcar_docker)  │
├─────────────────┤  ├─────────────────┤  ├─────────────────┤
│ • React 19      │  │ • Django 5.1    │  │ • ROS 2 Humble  │
│ • Router 7      │  │ • DRF + JWT     │  │ • Gazebo        │
│ • Three.js      │  │ • SQLite        │  │ • rosbridge     │
│ • Blockly       │  │ • CORS enabled  │  │ • Unity TCP     │
│ • Monaco Editor │  │ • Fixtures      │  │ • URDF/Meshes   │
│ • ROSLIB.js     │  │ • Progress API  │  │ • Static Server │
└─────────────────┘  └─────────────────┘  └─────────────────┘
```

### Three Main Components

1. **Frontend: AVEDU/avedu/** (React 19)
   - Port: 3000
   - Interactive learning UI with Monaco IDE, 3D visualizations, Blockly programming

2. **Backend: LAD/lad/** (Django 5.1)
   - Port: 8000
   - REST API for curriculum, user progress, authentication, file management

3. **ROS Environment: qcar_docker/** (ROS 2 Humble in Docker)
   - Port 9090: rosbridge WebSocket (web client communication)
   - Port 7000: Static file server (URDF/meshes with CORS)
   - Port 10000: ROS-TCP-Endpoint (Unity communication)
   - Port 8080: web_video_server (camera streams)

## Directory Structure

```
L.A.D/
├── AVEDU/avedu/                    # Frontend (React)
│   ├── src/
│   │   ├── components/             # Reusable UI components
│   │   │   ├── blocks/            # Blockly visual programming
│   │   │   ├── ide/               # Web IDE (Monaco, Terminal, File Explorer)
│   │   │   ├── visualization/     # LiDAR, 3D visualizations
│   │   │   ├── UnityWebGL.jsx     # Unity integration
│   │   │   └── InteractiveTutorial.jsx
│   │   ├── context/               # React Context providers
│   │   │   ├── AuthContext.jsx    # JWT authentication
│   │   │   └── ProgressContext.jsx # Learning progress
│   │   ├── hooks/
│   │   │   ├── useRoslib.js       # ROS WebSocket connection
│   │   │   └── useROS2Workspace.js # ROS workspace management
│   │   ├── levels/                # Learning level components
│   │   │   ├── Turtlesim.jsx
│   │   │   ├── GazeboSim.jsx
│   │   │   ├── Ros2*.jsx          # ROS 2 concept levels
│   │   │   ├── Sensing*.jsx       # Sensor levels
│   │   │   └── slides*/           # Slide-based lessons
│   │   ├── pages/
│   │   │   ├── Learn.jsx          # Course catalog
│   │   │   └── LearnLevel.jsx     # Individual lesson view
│   │   ├── services/
│   │   │   └── fileApi.js         # File system API client
│   │   └── App.js                 # Main routing
│   ├── package.json
│   └── Dockerfile
│
├── LAD/lad/                        # Backend (Django)
│   ├── apps/learning/             # Learning module
│   │   ├── models.py              # Unit, Level, Objective, Progress
│   │   ├── views.py               # API endpoints (ViewSets)
│   │   └── serializers.py
│   ├── workspace/                 # IDE/Workspace module
│   │   ├── models.py              # Canvas, WorkspaceFile, CustomMesh
│   │   └── views.py               # File CRUD, terminal execution
│   ├── core/
│   │   ├── settings.py            # Django configuration
│   │   ├── urls.py                # URL routing
│   │   └── ip_config.py           # Network config loader
│   ├── fixtures/
│   │   └── curriculum_data.json   # Learning content
│   ├── requirements.txt
│   └── manage.py
│
├── qcar_docker/                    # ROS 2 Docker Environment
│   ├── qcar/rosws/src/
│   │   ├── qcar_description/      # Robot URDF, meshes
│   │   └── qcar_bringup/          # Launch files
│   ├── Unity-Robotics-Hub-main/   # Unity integration
│   ├── worlds/                    # Gazebo world files
│   ├── docker-compose.yml
│   ├── entrypoint.sh              # URDF generation, CORS setup
│   └── http_cors.py               # Static file server
│
├── config/
│   └── ip_config.json             # Centralized network IP
│
├── scripts/
│   ├── start-all.bat/.sh          # Start all services
│   └── stop-all.bat/.sh           # Stop all services
│
├── k8s/                           # Kubernetes manifests
│   ├── deployment-*.yaml
│   ├── service-*.yaml
│   └── ingress.yaml
│
└── installer.bat/.sh              # Automated setup
```

## Key Technologies

### Frontend Stack
- **React 19.1.1** - UI framework
- **React Router 7.9.1** - Routing
- **Three.js + @react-three/fiber** - 3D visualization
- **roslib 1.4.1** - ROS WebSocket client
- **ros3d + urdf-loader** - ROS 3D visualization
- **Blockly** - Visual programming
- **Monaco Editor** - Code editor (VS Code engine)
- **xterm.js** - Terminal emulator
- **react-unity-webgl** - Unity integration

### Backend Stack
- **Django 5.1.3** - Web framework
- **Django REST Framework 3.15.2** - REST API
- **SimpleJWT 5.3.1** - JWT authentication
- **django-cors-headers** - CORS support
- **SQLite** - Database

### ROS 2 Stack
- **ROS 2 Humble** - Robotics middleware
- **Gazebo** - Physics simulation
- **rosbridge_server** - WebSocket bridge
- **ROS-TCP-Endpoint v0.6.0** - Unity communication
- **web_video_server** - Camera streaming
- **Custom HTTP server** - Static files with CORS

## Network Configuration

**Single Source of Truth**: `config/ip_config.json`
```json
{
  "exposed_ip": "192.168.72.6"
}
```

This IP is used by:
- Django: CORS/CSRF origins
- React: ROS WebSocket URL
- ROS Docker: CORS allow origin

**Ports**:
- 3000: React dev server
- 8000: Django API
- 9090: rosbridge WebSocket
- 7000: Static files (URDF/meshes)
- 10000: Unity TCP endpoint
- 8080: web_video_server

## Starting the Platform

### Development Mode (Automated)
```bash
# Windows
scripts\start-all.bat

# Linux/Mac
./scripts/start-all.sh
```

This script:
1. Auto-detects local IP
2. Updates config/ip_config.json
3. Configures React .env.local
4. Configures Docker .env
5. Starts ROS Docker containers
6. Starts Django backend
7. Starts React frontend

### Manual Development Mode
```bash
# 1. Start ROS Docker
cd qcar_docker
docker compose up -d

# 2. Start Django
cd LAD/lad
..\\.venv\Scripts\activate  # Windows
python manage.py migrate
python manage.py runserver 0.0.0.0:8000

# 3. Start React
cd AVEDU/avedu
npm start  # Runs on 0.0.0.0:3000
```

### Stopping Services
```bash
# Windows
scripts\stop-all.bat

# Linux/Mac
./scripts/stop-all.sh
```

## Data Models

### Learning Models (LAD/lad/apps/learning/models.py)

**Unit** - Course module
- slug (PK), title, order, is_active

**Level** - Individual lesson
- slug (PK), unit (FK), title, order, is_active

**Objective** - Specific learning goal
- level (FK), code (unique), description, points

**UserProgress** - Level completion tracking
- user (FK), level (FK), completed, score

**ObjectiveProgress** - Individual achievement
- user (FK), objective (FK), achieved, achieved_at

**UnitProgress** - Unit-level aggregation
- user (FK), unit (FK), completed, score

### Workspace Models (LAD/lad/workspace/models.py)

**Canvas** - User's ROS workspace
- id (UUID), user (FK), name, description
- Docker path: `/workspaces/{username}/{canvas_id}/`

**WorkspaceFile** - Files in canvas
- canvas (FK), path, file_type, content

**CustomMesh** - Uploaded 3D meshes
- canvas (FK), file (FileField), file_size, file_type

## API Endpoints

### Authentication
- `POST /api/token/` - Login (get JWT)
- `POST /api/token/refresh/` - Refresh token
- `POST /api/register/` - Student registration

### Learning
- `GET /api/units/` - List all units
- `GET /api/units/progress/me/` - Units with user progress
- `GET /api/levels/progress/me/` - Levels with user progress
- `POST /api/levels/{slug}/complete/` - Mark level complete
- `POST /api/objectives/{code}/hit/` - Mark objective achieved

### Workspace (IDE)
- `GET /api/workspace/canvases/` - List user canvases
- `POST /api/workspace/canvases/` - Create canvas
- `GET /api/workspace/canvases/{id}/tree/` - File tree
- `GET /api/workspace/canvases/{id}/files/{path}` - Read file
- `POST /api/workspace/canvases/{id}/files/` - Create/update file
- `POST /api/workspace/canvases/{id}/execute/` - Run terminal command

## Key Workflows

### 1. Authentication Flow
1. User enters credentials → AuthContext.jsx
2. POST /api/token/ → Django SimpleJWT
3. JWT token stored in localStorage
4. Token sent in Authorization header for all requests

### 2. ROS Integration Flow
1. useRoslib() hook creates ROSLIB.Ros instance
2. Connects to ws://<IP>:9090 (rosbridge_server)
3. Components create Topics/Services via ROSLIB
4. Publishing: React → rosbridge → ROS 2 topic
5. Subscribing: ROS topic → rosbridge → WebSocket → React
6. URDF loaded from http://<IP>:7000/qcar_description/urdf/

### 3. Progress Tracking Flow
1. User completes objective (e.g., robot reaches goal)
2. React calls /api/objectives/{code}/hit/
3. Django creates ObjectiveProgress record
4. Recalculates UserProgress.score
5. Frontend updates UI

### 4. IDE Workspace Flow
1. User creates canvas → POST /api/workspace/canvases/
2. Files saved → POST /api/workspace/canvases/{id}/files/
3. Backend writes to /workspaces/{username}/{canvas_id}/
4. Terminal commands executed in ROS Docker container
5. File explorer fetches tree from API

## Curriculum Structure

The platform teaches 11 units (from curriculum_data.json):
1. **Introduction** - Platform basics, UI navigation
2. **ROS 2 Concepts** - Nodes, topics, publishers/subscribers
3. **Sensing** - LiDAR, cameras, sensor integration
4. **ROS 2 Advanced** - QoS, real-time, algorithms
5. **Transformations** - TF, coordinate frames
6. **Perception** - Object detection, sensor fusion
7. **Simulation** - Gazebo, URDF, robot modeling
8. **Planning** - Path planning, motion control
9. **Control** - PID, model-based control
10. **AI** - Machine learning, neural networks
11. **Safety V&V** - Validation, verification

Each unit contains multiple levels with:
- Interactive slides
- ROS simulations (Turtlesim, Gazebo)
- Code exercises (Blockly or Python/C++)
- Automated objective validation

## Important Architectural Notes

1. **URDF Generation**: `entrypoint.sh` generates TWO URDFs:
   - `robot_runtime.urdf` - Relative paths for browser
   - `robot_gazebo.urdf` - Absolute paths for Gazebo

2. **Dual ROS Communication**:
   - rosbridge (9090) - Web clients
   - ROS-TCP-Endpoint (10000) - Unity

3. **Workspace Isolation**: Each user's files in `/workspaces/{username}/{canvas_id}/`

4. **JWT Configuration**: 5-day access tokens (long-lived for educational use)

5. **Static File Server**: Custom Python server (http_cors.py) with CORS

6. **Progress Auto-Calculation**: Django recalculates scores on objective achievement

## Deployment Options

1. **Local Development** - React dev server, Django runserver, ROS Docker
2. **LAN Deployment** - Single machine serves entire class
3. **Docker Compose** - docker-compose.server.yml (all services containerized)
4. **Kubernetes** - k8s/ manifests with ingress routing

## Common Tasks

### Adding a New Level
1. Create component in `AVEDU/avedu/src/levels/`
2. Add level to `LAD/lad/fixtures/curriculum_data.json`
3. Run `python manage.py load_curriculum`
4. Import component in `AVEDU/avedu/src/pages/LearnLevel.jsx`

### Debugging ROS Communication
1. Check rosbridge: `docker logs qcar-ros`
2. Check WebSocket: Browser DevTools → Network → WS
3. Verify IP in `config/ip_config.json`
4. Check CORS in Docker entrypoint logs

### Updating Curriculum
```bash
cd LAD/lad
python manage.py load_curriculum
```

### Resetting Database
```bash
cd LAD/lad
rm db.sqlite3
python manage.py migrate
python manage.py load_curriculum
```

## Git Status (Current)
Modified files:
- .claude/settings.local.json
- AVEDU/avedu/src/levels/slidesSensing/02a-LidarSubscriberInteractive.jsx
- AVEDU/avedu/src/pages/IDETestPage.jsx
- AVEDU/avedu/src/pages/LearnLevel.jsx
- AVEDU/avedu/src/services/fileApi.js
- LAD/lad/fixtures/curriculum_data.json
- LAD/lad/workspace/views.py
- config/ip_config.json
- qcar_docker/docker-compose.yml
- scripts/start-all.bat
- scripts/stop-all.bat

New untracked features:
- LiDAR visualization components
- ROS 2 Advanced levels (QoS, Realtime, Algorithms)
- Transformation (TF) levels
- File explorer caching implementation

## Troubleshooting

### Services won't start
- Check Docker Desktop is running
- Verify ports 3000, 8000, 9090 are free
- Check IP configuration in config/ip_config.json

### CORS errors
- Verify IP in config/ip_config.json matches detected IP
- Restart all services after IP change
- Check Django ALLOWED_HOSTS in settings.py

### ROS connection issues
- Verify rosbridge is running: `docker ps | grep qcar`
- Check WebSocket URL in browser console
- Test rosbridge: `docker exec -it qcar-ros ros2 topic list`

### File save errors in IDE
- Check Docker volume permissions
- Verify canvas exists: GET /api/workspace/canvases/
- Check Django logs for errors
