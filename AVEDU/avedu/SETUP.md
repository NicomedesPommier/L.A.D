# Development Setup - LAN Access

This setup allows the React development server to be accessible from other devices on your local network.

## How It Works

1. **Automatic IP Detection**: When you run `npm start`, the `detect-ip.js` script automatically:
   - Detects your computer's local network IP address
   - Updates `config/ip_config.json` with the detected IP
   - Configures the dev server to accept connections from any host

2. **Network Configuration**: The detected IP is shared across:
   - Frontend (React dev server)
   - Backend (Django CORS settings)
   - ROS Docker (rosbridge CORS)

3. **Dev Server Configuration**: Uses `react-app-rewired` to customize webpack dev server:
   - Binds to `0.0.0.0` (all network interfaces)
   - Sets `allowedHosts: 'all'` to accept LAN connections
   - Disables host checking for development

## Quick Start

```bash
# Install dependencies (includes react-app-rewired)
npm install

# Start development server (auto-detects IP and updates config)
npm start
```

The terminal will show:
```
🔍 Detected local IP: 192.168.x.x
✅ Updated config/ip_config.json with IP: 192.168.x.x

🌐 Your app will be accessible at:
   Local:   http://localhost:3000
   Network: http://192.168.x.x:3000
```

## Accessing from Other Devices

1. Make sure your computer and the other device are on the same network
2. Note the IP address shown in the terminal (e.g., `192.168.x.x`)
3. On the other device, open a browser and navigate to: `http://192.168.x.x:3000`

## Firewall Configuration

If other devices cannot connect, you may need to allow port 3000 through your firewall:

**Windows:**
```powershell
netsh advfirewall firewall add rule name="React Dev Server" dir=in action=allow protocol=TCP localport=3000
```

**Linux:**
```bash
sudo ufw allow 3000/tcp
```

**macOS:**
System Preferences → Security & Privacy → Firewall → Firewall Options → Add port 3000

## Files Created/Modified

- `scripts/detect-ip.js` - IP detection and config update script
- `config-overrides.js` - Webpack dev server customization
- `.env.local` - Local environment overrides (gitignored)
- `package.json` - Added `prestart` script and `react-app-rewired`

## Troubleshooting

**Error: "Invalid options object. Dev Server has been initialized..."**
- Run `npm install` to ensure `react-app-rewired` is installed
- Delete `node_modules` and `package-lock.json`, then run `npm install` again

**Cannot connect from other devices:**
- Check firewall settings (see above)
- Verify both devices are on the same network
- Try disabling VPN if active

**Wrong IP detected:**
- Manually edit `config/ip_config.json` with the correct IP
- Restart the dev server

## Production Builds

For production, the IP detection is not used. The build process remains standard:
```bash
npm run build
```

Deploy the `build/` folder to your production server and configure the actual production URLs in your hosting environment.
