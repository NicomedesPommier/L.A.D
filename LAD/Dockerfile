# =============================================================================
# L.A.D. Backend - Multi-stage Dockerfile
# Django REST API with JWT authentication
# =============================================================================

# -----------------------------------------------------------------------------
# Stage 1: Base
# -----------------------------------------------------------------------------
FROM python:3.11-slim AS base

# Prevent Python from writing pyc files and buffering stdout/stderr
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    tini \
    curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# -----------------------------------------------------------------------------
# Stage 2: Dependencies
# -----------------------------------------------------------------------------
FROM base AS deps

# Install build dependencies for some Python packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install dependencies
COPY lad/requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt \
    && pip install --no-cache-dir gunicorn psycopg2-binary

# -----------------------------------------------------------------------------
# Stage 3: Production
# -----------------------------------------------------------------------------
FROM base AS production

# Create non-root user for security
RUN groupadd -g 1001 django \
    && useradd -u 1001 -g django -s /bin/bash -m django

# Install runtime PostgreSQL client library
RUN apt-get update && apt-get install -y --no-install-recommends \
    libpq5 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy installed packages from deps stage
COPY --from=deps /usr/local/lib/python3.11/site-packages /usr/local/lib/python3.11/site-packages
COPY --from=deps /usr/local/bin /usr/local/bin

# Copy application code
COPY --chown=django:django lad/ ./

# Create directories for static files and database
RUN mkdir -p /app/static_collected /app/static/images /app/data \
    && chown -R django:django /app

# Copy entrypoint script
COPY --chown=django:django docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

# Switch to non-root user
USER django

# Expose port
EXPOSE 8000

# Environment
ENV DJANGO_SETTINGS_MODULE=core.settings
ENV PORT=8000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=10s --retries=3 \
    CMD curl -f http://localhost:8000/api/health/ || exit 1

# Use tini as init for proper signal handling
ENTRYPOINT ["/usr/bin/tini", "--", "/docker-entrypoint.sh"]

# Run with gunicorn
CMD ["gunicorn", "--bind", "0.0.0.0:8000", "--workers", "2", "--threads", "4", "core.wsgi:application"]
