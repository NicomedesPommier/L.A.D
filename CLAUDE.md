# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

L.A.D (Learn Autonomous Driving) is an educational platform for robotics and autonomous driving courses. It integrates three main components:

1. **React Frontend** (`AVEDU/avedu/`) - Student-facing web application for consuming learning units, levels, and ROS widgets
2. **Django REST Backend** (`LAD/lad/`) - Manages authentication, content catalog, and user progress
3. **ROS 2 Docker Environment** (`qcar_docker/`) - Provides rosbridge and QCar simulation for browser-based interactive robotics practice

## Architecture

The platform uses a three-tier architecture where:
- Frontend communicates with backend via REST API with JWT authentication
- Frontend connects to ROS 2 nodes via rosbridge WebSocket for real-time robot interaction
- All network configuration is centralized in `config/ip_config.json` for easy IP management across all services

## Development Commands

### Frontend (AVEDU/avedu/)
```bash
cd AVEDU/avedu
npm install                 # Install dependencies
npm start                   # Auto-detect IP, update config, start dev server on 0.0.0.0:3000
npm test                    # Run React tests (Jest + Testing Library)
npm run build               # Build production bundle
```

**Note:** `npm start` automatically runs `prestart` script which detects your local IP and updates `config/ip_config.json`

### Quick Start All Services

For convenience, use the provided scripts to start/stop everything:

**Windows:**
```cmd
scripts\start-all.bat       # Start backend, ROS docker, and frontend
scripts\stop-all.bat        # Stop all services
```

**Linux/Mac:**
```bash
chmod +x scripts/*.sh       # Make scripts executable (first time only)
./scripts/start-all.sh      # Start backend, ROS docker, and frontend
./scripts/stop-all.sh       # Stop all services
```

### Backend (LAD/lad/)
```bash
cd LAD/lad
python -m venv ../.venv     # Create virtual environment (if needed)
source ../.venv/bin/activate  # Activate on Linux/Mac
..\.venv\Scripts\activate   # Activate on Windows
pip install -r requirements.txt  # Install dependencies
python manage.py migrate    # Run database migrations
python manage.py createsuperuser  # Create admin user
python manage.py runserver 0.0.0.0:8000  # Start development server
```

### ROS 2 Docker (qcar_docker/)
```bash
cd qcar_docker
docker compose build        # Build ROS 2 image with QCar packages
docker compose up           # Start rosbridge (9090), static server (7000), optional WVS (8080)
docker compose down         # Stop and remove containers
```

## Network Configuration

All services read from `config/ip_config.json`:
```json
{
  "exposed_ip": "192.168.100.116"
}
```

### Automatic IP Detection

The frontend automatically detects and updates the IP when starting:
```bash
npm start  # Runs prestart script to detect IP and update config
```

The `scripts/detect-ip.js` script:
1. Detects the local network IP (first non-loopback IPv4 interface)
2. Updates `config/ip_config.json` with the detected IP
3. Displays the network URL for LAN access

### Manual IP Configuration

To manually set a specific IP:
```bash
node scripts/set-ip.js 192.168.1.100
```

When changing the deployment IP:
1. Either run `npm start` (auto-detects) or `node scripts/set-ip.js <IP>` (manual)
2. Restart all services (backend, frontend dev server, ROS docker)
3. The backend exposes this config at `/api/config/network/` for frontend consumption

### LAN Access Configuration

The development server is configured to accept connections from other devices on the local network:
- Uses `react-app-rewired` to customize webpack dev server
- Binds to `0.0.0.0` (all network interfaces)
- Sets `allowedHosts: 'all'` to bypass host checking
- Configuration in `config-overrides.js`

## Key Code Architecture

### Frontend (React)

**Routing Structure:**
- `/` - Home page
- `/learn` - Main learning interface
- `/learn/:unitSlug` - Unit page with level list
- `/learn/:unitSlug/:levelSlug` - Individual level content with ROS widgets

**Critical Files:**
- `src/App.js` - Main routing configuration
- `src/context/AuthContext.jsx` - JWT authentication and `apiFetch` wrapper
- `src/context/ProgressContext.jsx` - User progress management
- `src/hooks/useRoslib.js` - ROS WebSocket connection management with auto-reconnect
- `src/hooks/useStableRosSub.js` - Stable ROS topic subscription hook
- `src/config.js` - API and rosbridge URL configuration
- `src/ip.js` - Dynamic IP configuration fetcher
- `src/pages/Learn.jsx` - Fetches catalog and synchronizes progress
- `src/pages/LearnLevel.jsx` - Renders level content and coordinates ROS widgets
- `src/levels/` - Level-specific components and ROS widget implementations
- `scripts/detect-ip.js` - Auto-detect local network IP and update config
- `config-overrides.js` - Webpack dev server customization for LAN access
- `.env.local` - Local environment overrides (gitignored, auto-generated)

**ROS Integration:**
- The `useRoslib` hook encapsulates ROSLIB.js and provides: `connected`, `subscribeTopic()`, `advertise()`, `callService()`
- ROS URL is dynamically determined from network config or environment variables
- Connection is established once per component lifecycle with automatic cleanup

**Widget Development:**
- Place level-specific widgets in `src/levels/`
- Use `useRoslib` hook for ROS communication
- Mark objectives complete via backend API when ROS conditions are met
- Examples: `src/levels/RosBasic.jsx`, `src/levels/rviz.jsx`

### Backend (Django)

**Django Apps:**
- `core/` - Project settings, URL routing, network config utilities
- `apps/learning/` - Learning management app (units, levels, objectives, progress)

**Database Models:**
- `Unit` - Top-level course module (slug, title, order, is_active)
- `Level` - Individual lesson within a unit (slug, unit FK, title, order, is_active)
- `Objective` - Specific learning goal within a level (code, description, points, level FK)
- `UserProgress` - Tracks user completion per level (user FK, level FK, completed, score)
- `ObjectiveProgress` - Tracks individual objective completion (user FK, objective FK, achieved)
- `UnitProgress` - Aggregated progress per unit (user FK, unit FK, completed, score)

**API Endpoints:**
- `POST /api/token/` - JWT authentication (returns access + refresh tokens)
- `GET /api/units/` - List all units with nested levels
- `GET /api/levels/<slug>/` - Level detail
- `GET /api/levels/progress/me/` - Current user's progress
- `POST /api/objectives/complete/` - Mark objective as complete
- `GET /api/config/network/` - Network configuration (IP/URLs)

**Authentication:**
- Uses `rest_framework_simplejwt` for JWT tokens
- Access token lifetime: 5 days
- Refresh token lifetime: 90 days
- Frontend stores token in localStorage and includes in Authorization header

**CORS Configuration:**
- `CORS_ALLOWED_ORIGINS` includes localhost:3000 and dynamic IP from `ip_config.json`
- `CSRF_TRUSTED_ORIGINS` mirrors CORS origins
- Uses `corsheaders` middleware

### ROS 2 Docker Environment

**ROS Workspace:**
- Base image: `osrf/ros:humble-desktop`
- Workspace: `/ros2_ws/src/`
- Packages: `qcar_description`, `qcar_bringup`, `tf2_web_republisher_py`

**Services Exposed:**
- Port 9090: `rosbridge_server` WebSocket for frontend communication
- Port 7000: Static HTTP server for URDF files and meshes (with CORS)
- Port 8080: `web_video_server` (optional, controlled by `ENABLE_WVS`)

**entrypoint.sh Behavior:**
1. Reads `IP_CONFIG_PATH` (`/config/ip_config.json`) for network configuration
2. Generates URDF from Xacro with runtime parameters
3. Starts `http_cors.py` static server on port 7000
4. Launches `qcar_bringup/web_viz.launch.py` with configured CORS origins
5. Optionally starts `joint_state_publisher`, `rosapi`, `web_video_server`, `turtlesim`

**Environment Variables:**
- `IP_CONFIG_PATH` - Path to shared IP config JSON (mounted volume)
- `CORS_ALLOW_ORIGIN` - Comma-separated allowed origins (auto-derived if not set)
- `ENABLE_JSP`, `ENABLE_ROSAPI`, `ENABLE_WVS`, `ENABLE_TURTLESIM` - Feature toggles (1/0)
- `XACRO_ARGS` - Additional arguments for URDF generation
- `STATIC_PORT`, `WVS_PORT` - Port overrides for static and video servers

## Common Development Tasks

### Adding a New Level

1. Create objectives in Django admin (`/admin`)
2. Add level component in `src/levels/<unit>/<level>.jsx`
3. Implement ROS widgets using `useRoslib` hook
4. Call backend API to mark objectives complete when conditions met
5. Test full flow: frontend → backend → ROS → backend → frontend

### Changing Network IP

**Automatic (Recommended):**
```bash
cd AVEDU/avedu
npm start  # Automatically detects IP and updates config
```

**Manual:**
```bash
node scripts/set-ip.js 192.168.1.100
# Then restart all services
```

**After IP change:**
1. Restart Django: `python manage.py runserver 0.0.0.0:8000`
2. Restart frontend: `npm start`
3. Restart ROS Docker: `docker compose restart`

### Adding ROS Packages to Docker

1. Add packages to `qcar_docker/qcar/rosws/src/`
2. Update `Dockerfile` if external dependencies needed
3. Rebuild: `docker compose build`
4. Update launch files in `qcar_bringup` if needed

### Running Full Stack Locally

1. Start backend: `cd LAD/lad && python manage.py runserver 0.0.0.0:8000`
2. Start ROS Docker: `cd qcar_docker && docker compose up`
3. Start frontend: `cd AVEDU/avedu && npm start`
4. Access at `http://localhost:3000`

### Database Migrations

```bash
cd LAD/lad
python manage.py makemigrations    # Create new migrations
python manage.py migrate           # Apply migrations
python manage.py loaddata fixtures.json  # Load seed data (if exists)
```

## Testing

### Frontend Tests
- Framework: Jest + React Testing Library
- Run: `npm test` (watch mode) or `CI=true npm test` (single run)
- Focus on critical components, hooks, and ROS integration

### Backend Tests
- Framework: Django TestCase or pytest
- Run: `python manage.py test apps.learning`

## Important Notes

### Environment Variables

**Frontend (.env in AVEDU/avedu/):**
- `REACT_APP_API_BASE` - Backend API URL (default: `/api`)
- `REACT_APP_ROS_WS` - Rosbridge WebSocket URL (auto-derived if not set)
- Variables are baked into build at compile time - rebuild after changes

**Backend (LAD/lad/core/settings.py):**
- Reads `config/ip_config.json` via `load_network_config()`
- `DEBUG=True` in development - set to False in production
- `SECRET_KEY` should be changed for production deployments
- SQLite used for development - consider PostgreSQL for production

### Git and Repository Structure

- Main repository contains three subdirectories with their own purposes
- `LAD/lad/.venv/` and `AVEDU/avedu/node_modules/` are gitignored
- QCar ROS workspace is in `qcar_docker/qcar/rosws/src/`
- Images for documentation stored in `images/` directory

### CORS and Security

- Both rosbridge and Django backend require CORS configuration
- Frontend origin must be in both `CORS_ALLOWED_ORIGINS` (Django) and `CORS_ALLOW_ORIGIN` (rosbridge)
- Production deployments should use HTTPS and update WebSocket to WSS
- JWT tokens have long lifetimes for development - adjust for production

### ROS Integration Patterns

- Never create multiple ROS connections in a single component
- Use `useRoslib` hook at top level and pass down methods
- Always unsubscribe from topics in cleanup
- Handle connection state in UI (show loading/error when `connected` is false)
- Throttle topic subscriptions (default 50ms) to avoid overwhelming the browser

### Component Structure

- Blockly/flow-based editors in `src/components/blocks/`
- Turtlesim interface in `src/components/tsim/`
- Reusable widgets should be placed in `src/components/`
- Level-specific code stays in `src/levels/`
