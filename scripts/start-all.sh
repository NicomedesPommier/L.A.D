#!/bin/bash
# scripts/start-all.sh - Start all L.A.D services with auto-detected IP

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   L.A.D Platform - Starting Services  ║${NC}"
echo -e "${BLUE}╔════════════════════════════════════════╗${NC}\n"

# Detect IP using Node.js
echo -e "${GREEN}[1/4]${NC} Detecting local IP address..."
cd AVEDU/avedu
IP=$(node scripts/detect-ip.js 2>&1 | grep "Detected local IP:" | awk '{print $NF}')
cd ../..

if [ -z "$IP" ]; then
    echo -e "${YELLOW}⚠️  Could not auto-detect IP, using config file${NC}"
    IP=$(cat config/ip_config.json | grep exposed_ip | cut -d'"' -f4)
fi

echo -e "${GREEN}✓${NC} Using IP: ${BLUE}$IP${NC}\n"

# Start ROS Docker
echo -e "${GREEN}[2/4]${NC} Starting ROS 2 Docker (rosbridge + QCar)..."
cd qcar_docker
docker compose up -d
cd ..
echo -e "${GREEN}✓${NC} ROS services running on:"
echo -e "   - rosbridge: ws://$IP:9090"
echo -e "   - static server: http://$IP:7000\n"

# Start Django Backend
echo -e "${GREEN}[3/4]${NC} Starting Django backend..."
cd LAD/lad
if [ ! -d "../.venv" ]; then
    echo -e "${YELLOW}⚠️  Virtual environment not found, creating...${NC}"
    python3 -m venv ../.venv
    source ../.venv/bin/activate
    pip install -r requirements.txt 2>&1 | grep -E "Successfully|Requirement already"
else
    source ../.venv/bin/activate
fi

# Run migrations if needed
python manage.py migrate --no-input 2>&1 | grep -E "Applying|No migrations"

# Start Django in background
python manage.py runserver 0.0.0.0:8000 &
DJANGO_PID=$!
echo $DJANGO_PID > /tmp/lad_django.pid
cd ../..
echo -e "${GREEN}✓${NC} Django API running on: http://$IP:8000\n"

# Start React Frontend
echo -e "${GREEN}[4/4]${NC} Starting React frontend..."
cd AVEDU/avedu
npm start &
REACT_PID=$!
echo $REACT_PID > /tmp/lad_react.pid
cd ../..

echo -e "\n${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}   All services started successfully!   ${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}\n"

echo -e "${BLUE}🌐 Access your application:${NC}"
echo -e "   Local:   ${GREEN}http://localhost:3000${NC}"
echo -e "   Network: ${GREEN}http://$IP:3000${NC}\n"

echo -e "${YELLOW}ℹ️  To stop all services, run:${NC}"
echo -e "   ./scripts/stop-all.sh\n"

echo -e "${YELLOW}ℹ️  To view logs:${NC}"
echo -e "   Django:  tail -f LAD/lad/django.log"
echo -e "   React:   Check terminal output"
echo -e "   Docker:  docker compose -f qcar_docker/docker-compose.yml logs -f\n"

# Keep script running (optional)
# wait
