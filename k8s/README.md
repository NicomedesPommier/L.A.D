# L.A.D. Kubernetes Deployment

This directory contains Kubernetes manifests for deploying the L.A.D. (Learn Autonomous Driving) platform.

## Architecture

The deployment consists of three main components:

1. **Frontend** - React application serving the web UI
2. **Backend** - Django REST API with JWT authentication
3. **ROS** - ROS 2 Humble with Gazebo simulation

## Prerequisites

- Kubernetes cluster (1.25+)
- kubectl configured with cluster access
- Container registry (GitLab Registry, Docker Hub, etc.)
- Nginx Ingress Controller (for ingress routing)

## Quick Start

### 1. Build and Push Images

```bash
# Build all images
./deploy-to-server.sh build-all

# Push to registry
./deploy-to-server.sh push
```

### 2. Configure Secrets

Before deploying, create actual secrets:

```bash
# Create secrets with real values
kubectl create secret generic lad-secrets \
  --namespace=lad \
  --from-literal=DJANGO_SECRET_KEY='your-secure-secret-key'
```

### 3. Deploy to Cluster

```bash
# Apply all manifests
kubectl apply -f k8s/namespace.yaml
kubectl apply -f k8s/configmap.yaml
kubectl apply -f k8s/pvc.yaml
kubectl apply -f k8s/secrets.yaml  # Or use kubectl create secret
kubectl apply -f k8s/deployment-frontend.yaml
kubectl apply -f k8s/deployment-backend.yaml
kubectl apply -f k8s/deployment-ros.yaml
kubectl apply -f k8s/service-frontend.yaml
kubectl apply -f k8s/service-backend.yaml
kubectl apply -f k8s/service-ros.yaml
kubectl apply -f k8s/ingress.yaml
```

Or use the deploy script:

```bash
./deploy-to-server.sh deploy
```

## Files Overview

| File | Description |
|------|-------------|
| `namespace.yaml` | Creates the `lad` namespace |
| `configmap.yaml` | Application configuration |
| `secrets.yaml` | Secret template (DO NOT commit real secrets!) |
| `pvc.yaml` | Persistent volume claims for data storage |
| `deployment-frontend.yaml` | React frontend deployment |
| `deployment-backend.yaml` | Django backend deployment |
| `deployment-ros.yaml` | ROS 2 simulation deployment |
| `service-frontend.yaml` | Frontend ClusterIP service |
| `service-backend.yaml` | Backend ClusterIP service |
| `service-ros.yaml` | ROS ClusterIP service with session affinity |
| `ingress.yaml` | Nginx ingress for routing |

## Configuration

### Update Registry Path

Before deploying, update the image paths in deployment files:

```yaml
# Change from:
image: registry.gitlab.com/YOUR_GROUP/lad-frontend:latest

# To your actual registry:
image: registry.gitlab.com/your-group/lad-frontend:latest
```

### Update Ingress Host

Edit `ingress.yaml` to set your domain:

```yaml
rules:
  - host: lad.yourdomain.com
```

### Resource Limits

Adjust resource limits based on your cluster capacity:

| Component | CPU Request | CPU Limit | Memory Request | Memory Limit |
|-----------|-------------|-----------|----------------|--------------|
| Frontend | 100m | 500m | 128Mi | 256Mi |
| Backend | 100m | 500m | 256Mi | 512Mi |
| ROS | 500m | 2000m | 1Gi | 4Gi |

## Monitoring

### Check Pod Status

```bash
kubectl -n lad get pods
kubectl -n lad describe pod <pod-name>
```

### View Logs

```bash
# Frontend logs
kubectl -n lad logs -f deployment/lad-frontend

# Backend logs
kubectl -n lad logs -f deployment/lad-backend

# ROS logs
kubectl -n lad logs -f deployment/lad-ros
```

### Check Services

```bash
kubectl -n lad get services
kubectl -n lad get ingress
```

## Troubleshooting

### Pod Not Starting

1. Check events: `kubectl -n lad describe pod <pod-name>`
2. Check logs: `kubectl -n lad logs <pod-name>`
3. Verify image pull: Ensure registry credentials are configured

### Database Issues

The backend uses SQLite by default in the persistent volume. For production, consider:

1. Using PostgreSQL as a separate deployment
2. Updating `DATABASE_URL` in secrets
3. Adding `psycopg2-binary` to requirements (already included in Dockerfile)

### ROS Not Connecting

1. Verify rosbridge is running: `kubectl -n lad logs deployment/lad-ros | grep rosbridge`
2. Check WebSocket connectivity through ingress
3. Ensure session affinity is working for WebSocket connections

## CI/CD Integration

The project includes GitLab CI/CD configuration (`.gitlab-ci.yml`) that:

1. Builds Docker images for all components
2. Pushes to GitLab Container Registry
3. Deploys to Kubernetes (staging/production)

Configure these CI/CD variables in GitLab:

- `KUBE_SERVER` - Kubernetes API server URL
- `KUBE_TOKEN` - Service account token
- `KUBE_CA_CERT` - Cluster CA certificate

## Alternative: Docker Compose

For simpler deployments without Kubernetes, use:

```bash
docker compose -f docker-compose.server.yml up -d
```
