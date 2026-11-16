#!/usr/bin/env bash
# Autonomous Perimeter Rover - Live Bridge Tunnel Launcher
# Exposes local ROS 2 Web Controller (http://localhost:5000) over a secure Cloudflare HTTPS Tunnel

echo "========================================================================"
echo "🚀 STARTING SECURE CLOUDFLARE HTTPS TUNNEL TO ROVER MISSION CONTROL"
echo "   Forwarding http://localhost:5000 to public HTTPS..."
echo "========================================================================"

CLOUDFLARED_BIN="/tmp/cloudflared"
if [ ! -f "$CLOUDFLARED_BIN" ]; then
    echo "Downloading cloudflared..."
    curl -s -L -o /tmp/cloudflared https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64
    chmod +x /tmp/cloudflared
fi

echo "Tunnel starting. Look for the 'https://...trycloudflare.com' URL below:"
echo "Copy that URL and paste it into the 'LIVE ROVER BRIDGE' input on your Streamlit app!"
echo "========================================================================"
exec /tmp/cloudflared tunnel --url http://127.0.0.1:5000
