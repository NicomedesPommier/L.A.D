# L.A.D Frontend - Installation & Setup Guide

This guide will help you set up the React frontend for LAN access, allowing other devices on your network to connect to the development server.

## Prerequisites

- Node.js >= 18.x
- npm >= 9.x
- Access to your local network (WiFi/Ethernet)

## Quick Start

### 1. Install Dependencies

```bash
cd AVEDU/avedu
npm install
```

This will install all required packages including:
- `react-app-rewired` - For customizing webpack config
- All React dependencies
- ROS integration libraries (roslib, ros3d, urdf-loader)
- UI libraries (blockly, three.js, react-three-fiber)

### 2. Start Development Server

```bash
npm start
```

This command will:
1. **Auto-detect your local network IP** (e.g., 192.168.1.100)
2. **Update `config/ip_config.json`** with the detected IP
3. **Start the dev server** on `0.0.0.0:3000` (accessible from LAN)
4. **Display access URLs** in the terminal

Expected output:
```
🔍 Detected local IP: 192.168.1.100
✅ Updated config/ip_config.json with IP: 192.168.1.100

🌐 Your app will be accessible at:
   Local:   http://localhost:3000
   Network: http://192.168.1.100:3000

Compiled successfully!

You can now view avedu in the browser.

  Local:            http://localhost:3000
  On Your Network:  http://192.168.1.100:3000
```

### 3. Access from Other Devices

On another device (phone, tablet, another computer) connected to the same network:

1. Open a web browser
2. Navigate to: `http://192.168.1.100:3000` (use your actual IP)
3. The application should load

## Troubleshooting

### Error: "Invalid options object. Dev Server has been initialized..."

**Solution:**
```bash
# Delete node_modules and reinstall
rm -rf node_modules package-lock.json
npm install
npm start
```

### Cannot Connect from Other Devices

**Check Firewall (Windows):**
```powershell
# Run as Administrator
netsh advfirewall firewall add rule name="React Dev Server" dir=in action=allow protocol=TCP localport=3000
```

**Check Firewall (Linux):**
```bash
sudo ufw allow 3000/tcp
sudo ufw reload
```

**Check Firewall (macOS):**
1. Open System Preferences → Security & Privacy
2. Click Firewall → Firewall Options
3. Add port 3000 or allow Node.js

**Other checks:**
- Ensure both devices are on the same WiFi network
- Try disabling VPN on both devices
- Check if antivirus is blocking connections
- Verify IP address with `ipconfig` (Windows) or `ifconfig` (Linux/Mac)

### Wrong IP Detected

**Option 1: Manual Override**
```bash
# Set a specific IP
node scripts/set-ip.js 192.168.1.100
npm start
```

**Option 2: Edit Config Directly**
Edit `config/ip_config.json`:
```json
{
  "exposed_ip": "192.168.1.100"
}
```
Then restart: `npm start`

### Environment Variables Not Working

Remember: `.env` variables are **baked into the build** at compile time.

**For development:**
- Edit `.env.local` (auto-created by `npm start`)
- Changes require restarting the dev server

**For production:**
- Edit `.env.production` or set environment variables before build
- Run `npm run build` after changes

## Configuration Files

### `.env.local` (Auto-generated, gitignored)
```bash
HOST=0.0.0.0
DANGEROUSLY_DISABLE_HOST_CHECK=true
REACT_APP_API_BASE=/api
```

### `config-overrides.js` (Webpack customization)
Configures webpack dev server to:
- Bind to all network interfaces (`0.0.0.0`)
- Allow connections from any host (`allowedHosts: 'all'`)
- Enable CORS for development

### `scripts/detect-ip.js`
- Automatically detects local network IP
- Updates `config/ip_config.json`
- Runs before `npm start` (via `prestart` script)

## Manual IP Management

### Detect and Update IP
```bash
node scripts/detect-ip.js
```

### Set Specific IP
```bash
node scripts/set-ip.js 192.168.1.100
```

### View Current IP
```bash
cat config/ip_config.json
```

## Development Workflow

### Standard Development (Local Only)
```bash
npm start
# Access at http://localhost:3000
```

### LAN Development (Multiple Devices)
```bash
npm start
# Access at http://192.168.x.x:3000 from any device on network
```

### Production Build
```bash
npm run build
# Deploy build/ folder to production server
```

## Network Architecture

```
┌─────────────────────────────────────────────────────────┐
│  Your Computer (192.168.1.100)                          │
│                                                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  React       │  │  Django      │  │  ROS Docker  │  │
│  │  :3000       │  │  :8000       │  │  :9090       │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
│         │                 │                 │            │
└─────────┼─────────────────┼─────────────────┼────────────┘
          │                 │                 │
          │  Local Network (192.168.1.x/24)  │
          │                 │                 │
    ┌─────┴─────┬───────────┴───────┬─────────┴─────┐
    │           │                   │               │
┌───▼───┐  ┌────▼────┐         ┌────▼────┐     ┌───▼────┐
│ Phone │  │ Tablet  │         │ Laptop  │     │  PC    │
└───────┘  └─────────┘         └─────────┘     └────────┘

All devices access:
- Frontend:  http://192.168.1.100:3000
- API:       http://192.168.1.100:8000/api
- ROS:       ws://192.168.1.100:9090
```

## Security Note

The `DANGEROUSLY_DISABLE_HOST_CHECK` setting is **only for development**. It bypasses webpack's host header check to allow LAN connections.

**Never use this in production!** Production builds don't use the dev server, so this setting doesn't apply.

## Next Steps

After the frontend is running:

1. **Start Backend:** See `LAD/lad/README.md`
2. **Start ROS Docker:** See `qcar_docker/README.md`
3. **Full Stack:** Use `scripts/start-all.bat` (Windows) or `scripts/start-all.sh` (Linux/Mac)

## Support

For issues or questions:
- Check `AVEDU/avedu/SETUP.md` for detailed setup information
- Review `CLAUDE.md` in the repository root for architecture details
- Check Django backend logs for API errors
- Use browser DevTools to debug frontend issues
