# File Save Error Troubleshooting Guide

## Error: "Failed to save file: Internal Server Error"

This error occurs when the backend cannot write files to the Docker container. Here's how to diagnose and fix it:

---

## ✅ Step 1: Check if Docker Container is Running

The backend needs a running Docker container named `qcar_docker-ros-1`.

### Check Container Status:
```bash
docker ps
```

**Expected output:**
```
CONTAINER ID   IMAGE         STATUS        NAMES
abc123def456   ros:humble    Up 2 hours    qcar_docker-ros-1
```

### ❌ If Container is NOT Running:
```bash
# Start the container
docker-compose up -d

# Or if using docker run
docker start qcar_docker-ros-1
```

---

## ✅ Step 2: Verify Container Name

The backend expects the container name to be **exactly** `qcar_docker-ros-1`.

### Check your actual container name:
```bash
docker ps --format "{{.Names}}"
```

### If container has a different name:

**Option A - Rename the container:**
```bash
# Stop container
docker stop YOUR_CONTAINER_NAME

# Rename it
docker rename YOUR_CONTAINER_NAME qcar_docker-ros-1

# Start it
docker start qcar_docker-ros-1
```

**Option B - Update backend settings:**

Edit `LAD/lad/workspace/views.py` line 22:
```python
# Change from:
DOCKER_CONTAINER = "qcar_docker-ros-1"

# To your actual container name:
DOCKER_CONTAINER = "your-actual-container-name"
```

Then restart Django:
```bash
python LAD/lad/manage.py runserver
```

---

## ✅ Step 3: Check Django Backend Logs

The backend now has detailed logging. Check your Django console/terminal for errors.

### Look for messages like:
```
[WorkspaceFile] Updating file: my_script.py
[Docker] Writing to: /workspaces/username/123/my_script.py
[Docker] ❌ Docker container 'qcar_docker-ros-1' is not running!
```

Or:
```
[Docker] ❌ CalledProcessError: permission denied
```

Or:
```
[Docker] ❌ UnicodeDecodeError: File contains invalid UTF-8
```

These detailed logs will tell you exactly what's wrong.

---

## ✅ Step 4: Test Docker Connection Manually

Try writing a file manually to verify Docker connectivity:

```bash
# Test if you can write to the container
echo "test" | docker exec -i qcar_docker-ros-1 tee /tmp/test.txt

# If that works, try creating a workspace directory
docker exec qcar_docker-ros-1 mkdir -p /workspaces/test

# Try writing a file there
echo "hello" | docker exec -i qcar_docker-ros-1 tee /workspaces/test/hello.txt
```

### If these commands fail:
- Docker daemon might not be running
- Container might not have write permissions
- Volume mount might be misconfigured

---

## ✅ Step 5: Check File Permissions

If Docker commands work but file save fails:

```bash
# Check permissions in container
docker exec qcar_docker-ros-1 ls -la /workspaces

# Fix permissions if needed
docker exec qcar_docker-ros-1 chmod -R 777 /workspaces
```

---

## ✅ Step 6: Verify API Connectivity

Test if the backend API is reachable:

### Check Backend Status:
1. Open browser console (F12)
2. Check Network tab when saving a file
3. Look for failed requests to `/api/workspace/...`

### Common Issues:
- Backend not running (should be at `http://localhost:8000`)
- Wrong API_BASE configuration
- CORS errors

---

## 🔍 Common Error Messages & Solutions

### Error: "Docker container 'qcar_docker-ros-1' is not running!"
**Solution:** Start the container (see Step 1)

### Error: "Available containers: None"
**Solution:** No Docker containers running at all
```bash
docker-compose up -d
```

### Error: "Permission denied"
**Solution:** Fix container permissions
```bash
docker exec qcar_docker-ros-1 chmod -R 777 /workspaces
```

### Error: "File contains invalid UTF-8 characters"
**Solution:** Your file has special characters that can't be encoded
- Try saving with plain ASCII text only
- Check for unusual emoji or special symbols

### Error: "No such file or directory"
**Solution:** Parent directory doesn't exist
- Backend should create it automatically now
- If error persists, check Docker logs

---

## 🛠️ Quick Diagnostic Commands

Run these to get a full diagnostic report:

```bash
# 1. Check Docker daemon
docker --version
docker ps

# 2. Check specific container
docker inspect qcar_docker-ros-1

# 3. Check workspace directory
docker exec qcar_docker-ros-1 ls -la /workspaces

# 4. Check backend API
curl http://localhost:8000/api/workspace/canvases/

# 5. Check Django logs
# Look at your Django terminal for [Docker] and [WorkspaceFile] messages
```

---

## 📝 Get Detailed Error Info

After you get the error, check these places:

### 1. Browser Console (F12)
- Look for `[FileAPI] ❌` messages
- Check Network tab for failed requests
- Look at request/response details

### 2. Django Terminal
- Look for `[WorkspaceFile]` messages
- Look for `[Docker]` messages
- Full stack traces are now printed

### 3. Error Alert Dialog
- Now shows specific error message
- Includes troubleshooting tips
- Tells you to check console

---

## 🎯 Expected Flow (When Working)

**Successful save should show:**

### Browser Console:
```
[IDE] Saving file: my_script.py Mode: text Content length: 1234
[IDE] Updating existing file with ID: 456
[IDE] ✓ File updated in database
[IDE] ✓ File saved successfully: my_script.py
```

### Django Terminal:
```
[WorkspaceFile] Updating file: my_script.py
[WorkspaceFile] Docker path: /workspaces/username/123/my_script.py
[WorkspaceFile] Content length: 1234
[Docker] Writing to: /workspaces/username/123/my_script.py
[Docker] Creating parent dir: /workspaces/username/123
[Docker] Writing 1234 bytes to file
[Docker] ✓ File written successfully
[WorkspaceFile] ✓ File updated successfully: my_script.py
```

---

## 💡 Still Having Issues?

1. **Enable verbose logging:**
   - Check `docker logs qcar_docker-ros-1`
   - Check Django's DEBUG mode is True

2. **Try a clean restart:**
   ```bash
   # Stop everything
   docker-compose down

   # Clear Docker volumes (WARNING: deletes data)
   docker volume prune

   # Start fresh
   docker-compose up -d
   ```

3. **Test with a simple file:**
   - Try saving a file with just "hello" as content
   - If that works, issue might be with file content/encoding

4. **Check your setup:**
   - Verify docker-compose.yml volume mounts
   - Check if container has correct volume permissions
   - Ensure workspace directories exist in container

---

## 📞 Report the Error

If none of this helps, please report with:

1. **Django console output** (the `[Docker]` and `[WorkspaceFile]` messages)
2. **Browser console output** (the `[FileAPI]` and `[IDE]` messages)
3. **Output of:** `docker ps` and `docker inspect qcar_docker-ros-1`
4. **The exact error message** from the alert dialog

---

*Last updated: 2025-12-29*
