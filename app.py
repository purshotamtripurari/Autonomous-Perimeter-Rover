#!/usr/bin/env python3
"""
Autonomous Perimeter Rover - Mission Command & Security Teleoperation Dashboard
Streamlit Cloud Web Application
Renders the full Mission Control GCS Console matching the local Gazebo/ROS 2 interface:
- Dual Active Control: Autonomous Patrol vs Manual Joystick Teleop
- Interactive Analog Joystick & Keyboard WASD
- Emergency Stop & Origin Reset Routine
- Real-time Rover Kinematics & Velocity Gauges
- Live Perimeter Detection & Intruder Audit Log with Filters, Search & CSV Export
- Embedded Tactical Surveillance Stream HUD
"""

import json
import os
import streamlit as st
import streamlit.components.v1 as components

# -----------------------------------------------------------------------------
# 1. STREAMLIT FULL-BLEED PAGE CONFIG
# -----------------------------------------------------------------------------
st.set_page_config(
    page_title="Autonomous Perimeter Rover - Mission Control",
    page_icon="🛡️",
    layout="wide",
    initial_sidebar_state="collapsed",
)

# Strip away Streamlit default padding, menu, headers, and footer
st.markdown(
    """
    <style>
    #MainMenu { display: none !important; }
    footer { display: none !important; }
    header { display: none !important; }
    [data-testid="stHeader"] { display: none !important; }
    [data-testid="stToolbar"] { display: none !important; }
    [data-testid="stSidebar"] { display: none !important; }
    .stApp {
        background-color: #0a0e14 !important;
        margin: 0 !important;
        padding: 0 !important;
    }
    .block-container {
        padding: 0 !important;
        margin: 0 !important;
        max-width: 100vw !important;
    }
    iframe {
        border: none !important;
        width: 100% !important;
        min-height: 100vh !important;
    }
    </style>
    """,
    unsafe_allow_html=True,
)

# -----------------------------------------------------------------------------
# 2. LOAD AUDIT LOG DATA (700+ RECORDED EVENTS)
# -----------------------------------------------------------------------------
DATA_PATH = os.path.join(os.path.dirname(__file__), "perimeter_detections.json")
raw_detections = []
if os.path.exists(DATA_PATH):
    try:
        with open(DATA_PATH, "r") as f:
            raw_detections = json.load(f)
            if not isinstance(raw_detections, list):
                raw_detections = []
    except Exception:
        raw_detections = []

json_detections_str = json.dumps(raw_detections)


# -----------------------------------------------------------------------------
# 3. BUILD COMPLETE MISSION CONTROL HTML DASHBOARD (PICTURE 2 SPEC)
# -----------------------------------------------------------------------------
DASHBOARD_HTML = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Autonomous Perimeter Rover - Mission Control</title>
  <style>
    :root {{
      --bg-dark: #0a0e14;
      --card-bg: #121822;
      --card-border: #1e293b;
      --card-hover: #172230;
      --text-main: #f1f5f9;
      --text-muted: #94a3b8;
      --accent-cyan: #00f0ff;
      --accent-green: #00ff88;
      --accent-yellow: #ffb700;
      --accent-red: #ff3366;
      --accent-blue: #38bdf8;
      --font-mono: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
    }}

    * {{
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      user-select: none;
    }}

    body {{
      background-color: var(--bg-dark);
      color: var(--text-main);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      overflow-x: hidden;
    }}

    /* TOP HEADER */
    header {{
      background: linear-gradient(180deg, #161f2e 0%, #0f1520 100%);
      border-bottom: 1px solid var(--card-border);
      padding: 12px 24px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      flex-wrap: wrap;
      gap: 16px;
      position: sticky;
      top: 0;
      z-index: 100;
    }}

    .brand {{
      display: flex;
      align-items: center;
      gap: 12px;
    }}

    .brand-icon {{
      width: 36px;
      height: 36px;
      background: radial-gradient(circle, var(--accent-cyan) 0%, rgba(0, 240, 255, 0.2) 70%);
      border: 2px solid var(--accent-cyan);
      border-radius: 8px;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 20px;
      box-shadow: 0 0 12px rgba(0, 240, 255, 0.4);
    }}

    .brand-title h1 {{
      font-size: 1.15rem;
      font-weight: 700;
      letter-spacing: 0.5px;
      color: #fff;
      display: flex;
      align-items: center;
      gap: 8px;
    }}

    .brand-title p {{
      font-size: 0.75rem;
      color: var(--text-muted);
      letter-spacing: 0.8px;
      font-family: var(--font-mono);
      text-transform: uppercase;
    }}

    /* MODE TOGGLE SWITCH PILL */
    .mode-switch-container {{
      display: flex;
      align-items: center;
      background: #090d14;
      border: 1px solid var(--card-border);
      border-radius: 30px;
      padding: 4px;
      gap: 4px;
    }}

    .mode-btn {{
      padding: 8px 18px;
      border: none;
      border-radius: 24px;
      font-size: 0.82rem;
      font-weight: 600;
      letter-spacing: 0.5px;
      cursor: pointer;
      display: flex;
      align-items: center;
      gap: 8px;
      transition: all 0.2s ease;
      color: var(--text-muted);
      background: transparent;
    }}

    .mode-btn.active-auto {{
      background: linear-gradient(135deg, #059669 0%, #10b981 100%);
      color: #fff;
      box-shadow: 0 0 14px rgba(16, 185, 129, 0.4);
    }}

    .mode-btn.active-manual {{
      background: linear-gradient(135deg, #d97706 0%, #f59e0b 100%);
      color: #fff;
      box-shadow: 0 0 14px rgba(245, 158, 11, 0.4);
    }}

    /* EMERGENCY STOP BUTTON */
    .estop-btn {{
      background: linear-gradient(135deg, #dc2626 0%, #ef4444 100%);
      color: #fff;
      border: 2px solid #b91c1c;
      border-radius: 8px;
      padding: 9px 20px;
      font-size: 0.85rem;
      font-weight: 700;
      letter-spacing: 0.8px;
      cursor: pointer;
      display: flex;
      align-items: center;
      gap: 10px;
      box-shadow: 0 0 16px rgba(239, 68, 68, 0.5);
      transition: all 0.2s ease;
      text-transform: uppercase;
    }}

    .estop-btn:hover {{
      background: linear-gradient(135deg, #b91c1c 0%, #dc2626 100%);
      transform: scale(1.02);
      box-shadow: 0 0 22px rgba(239, 68, 68, 0.8);
    }}

    .estop-btn.active-stopped {{
      background: #7f1d1d;
      border-color: #ef4444;
      animation: estop-pulse 1s infinite alternate;
    }}

    @keyframes estop-pulse {{
      from {{ box-shadow: 0 0 12px rgba(239, 68, 68, 0.5); }}
      to {{ box-shadow: 0 0 28px rgba(239, 68, 68, 0.9); }}
    }}

    /* SYSTEM STATUS PILL & CLOCK */
    .sys-status {{
      display: flex;
      align-items: center;
      gap: 16px;
      font-family: var(--font-mono);
      font-size: 0.8rem;
    }}

    .status-pill {{
      display: flex;
      align-items: center;
      gap: 6px;
      background: #090e16;
      border: 1px solid rgba(0, 255, 136, 0.3);
      padding: 5px 12px;
      border-radius: 20px;
      color: var(--accent-green);
      font-weight: 600;
    }}

    .pulse-dot {{
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: var(--accent-green);
      box-shadow: 0 0 8px var(--accent-green);
      animation: pulse 1.5s infinite;
    }}

    @keyframes pulse {{
      0% {{ opacity: 1; transform: scale(1); }}
      50% {{ opacity: 0.4; transform: scale(0.85); }}
      100% {{ opacity: 1; transform: scale(1); }}
    }}

    /* MAIN CONTENT LAYOUT */
    main {{
      padding: 20px 24px;
      display: flex;
      flex-direction: column;
      gap: 20px;
      flex: 1;
    }}

    /* TOP DECK: 3 COLUMN GRID */
    .top-deck-grid {{
      display: grid;
      grid-template-columns: 1fr 1.25fr 0.95fr;
      gap: 20px;
    }}

    @media (max-width: 1200px) {{
      .top-deck-grid {{
        grid-template-columns: 1fr 1fr;
      }}
    }}

    @media (max-width: 800px) {{
      .top-deck-grid {{
        grid-template-columns: 1fr;
      }}
    }}

    .card {{
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 12px;
      padding: 16px;
      display: flex;
      flex-direction: column;
      gap: 12px;
      box-shadow: 0 4px 16px rgba(0, 0, 0, 0.4);
    }}

    .card-header {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      border-bottom: 1px solid rgba(255, 255, 255, 0.05);
      padding-bottom: 10px;
    }}

    .card-title {{
      font-size: 0.88rem;
      font-weight: 700;
      letter-spacing: 0.5px;
      text-transform: uppercase;
      display: flex;
      align-items: center;
      gap: 8px;
      color: #e2e8f0;
    }}

    .card-badge {{
      font-family: var(--font-mono);
      font-size: 0.7rem;
      background: #090e16;
      border: 1px solid var(--card-border);
      padding: 3px 8px;
      border-radius: 4px;
      color: var(--accent-cyan);
    }}

    /* VIDEO STREAM CONTAINER */
    .video-container {{
      position: relative;
      width: 100%;
      height: 250px;
      background: #000;
      border-radius: 8px;
      overflow: hidden;
      border: 1px solid var(--card-border);
      display: flex;
      align-items: center;
      justify-content: center;
    }}

    .video-container img, .video-container svg {{
      width: 100%;
      height: 100%;
      object-fit: cover;
    }}

    .video-overlay-hud {{
      position: absolute;
      top: 10px;
      left: 10px;
      display: flex;
      gap: 8px;
      z-index: 10;
    }}

    .hud-chip {{
      font-family: var(--font-mono);
      font-size: 0.68rem;
      font-weight: 700;
      background: rgba(10, 14, 20, 0.85);
      border: 1px solid rgba(0, 240, 255, 0.3);
      color: var(--accent-cyan);
      padding: 3px 8px;
      border-radius: 4px;
      backdrop-filter: blur(4px);
    }}

    .checkpoint-tracker {{
      display: grid;
      grid-template-columns: repeat(6, 1fr);
      gap: 6px;
      margin-top: 4px;
    }}

    .wp-pill {{
      text-align: center;
      padding: 6px 4px;
      font-size: 0.72rem;
      font-family: var(--font-mono);
      font-weight: 700;
      background: #090e16;
      border: 1px solid var(--card-border);
      border-radius: 6px;
      color: var(--text-muted);
      transition: all 0.2s ease;
    }}

    .wp-pill.active {{
      background: rgba(0, 240, 255, 0.15);
      border-color: var(--accent-cyan);
      color: var(--accent-cyan);
      box-shadow: 0 0 10px rgba(0, 240, 255, 0.3);
    }}

    /* TELEMETRY DECK */
    .telemetry-deck {{
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      flex: 1;
    }}

    .telemetry-card {{
      background: #090e16;
      border: 1px solid var(--card-border);
      border-radius: 8px;
      padding: 12px;
      display: flex;
      flex-direction: column;
      gap: 8px;
    }}

    .telemetry-label {{
      font-size: 0.72rem;
      color: var(--text-muted);
      text-transform: uppercase;
      font-weight: 600;
      letter-spacing: 0.5px;
      display: flex;
      justify-content: space-between;
    }}

    .telemetry-value-box {{
      display: flex;
      align-items: baseline;
      gap: 6px;
    }}

    .telemetry-num {{
      font-family: var(--font-mono);
      font-size: 1.65rem;
      font-weight: 700;
      color: #fff;
    }}

    .telemetry-unit {{
      font-family: var(--font-mono);
      font-size: 0.75rem;
      color: var(--text-muted);
    }}

    .telemetry-bar-wrap {{
      width: 100%;
      height: 6px;
      background: #1a2332;
      border-radius: 3px;
      overflow: hidden;
    }}

    .telemetry-bar-fill {{
      height: 100%;
      border-radius: 3px;
      transition: width 0.2s ease;
    }}

    .bar-cyan {{ background: var(--accent-cyan); width: 25%; }}
    .bar-green {{ background: var(--accent-green); width: 10%; }}
    .bar-yellow {{ background: var(--accent-yellow); width: 50%; }}
    .bar-blue {{ background: var(--accent-blue); width: 40%; }}

    .telemetry-subtext {{
      font-size: 0.68rem;
      color: var(--text-muted);
      font-family: var(--font-mono);
    }}

    .quick-actions {{
      display: flex;
      gap: 8px;
      margin-top: 4px;
    }}

    .btn-action {{
      flex: 1;
      padding: 8px 12px;
      border-radius: 6px;
      border: 1px solid var(--card-border);
      background: #16202e;
      color: #e2e8f0;
      font-size: 0.75rem;
      font-weight: 600;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 6px;
      transition: all 0.15s ease;
    }}

    .btn-action:hover {{
      background: #1e2c3f;
      border-color: #38bdf8;
      color: #fff;
    }}

    .btn-resume {{
      background: rgba(16, 185, 129, 0.2);
      border-color: rgba(16, 185, 129, 0.4);
      color: var(--accent-green);
    }}

    /* JOYSTICK PANEL */
    .joystick-panel {{
      position: relative;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      height: 100%;
      min-height: 250px;
    }}

    .joystick-overlay-lock {{
      position: absolute;
      inset: 0;
      background: rgba(10, 14, 20, 0.90);
      border-radius: 8px;
      backdrop-filter: blur(4px);
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      gap: 12px;
      z-index: 20;
      padding: 20px;
      text-align: center;
      transition: opacity 0.2s ease, visibility 0.2s ease;
    }}

    .joystick-overlay-lock.hidden {{
      opacity: 0;
      visibility: hidden;
      pointer-events: none;
    }}

    .joystick-wrapper {{
      width: 200px;
      height: 200px;
      position: relative;
      display: flex;
      align-items: center;
      justify-content: center;
    }}

    #joystickCanvas {{
      background: transparent;
      cursor: grab;
      touch-action: none;
    }}

    #joystickCanvas:active {{
      cursor: grabbing;
    }}

    .joystick-readout {{
      display: flex;
      justify-content: space-between;
      width: 100%;
      margin-top: 10px;
      padding: 6px 12px;
      background: #090e16;
      border-radius: 6px;
      border: 1px solid var(--card-border);
      font-family: var(--font-mono);
      font-size: 0.72rem;
      color: var(--accent-cyan);
    }}

    /* DETECTION TABLE CARD */
    .table-card {{
      flex: 1;
    }}

    .table-toolbar {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 12px;
      flex-wrap: wrap;
    }}

    .filter-pills {{
      display: flex;
      gap: 6px;
      flex-wrap: wrap;
    }}

    .filter-btn {{
      padding: 5px 12px;
      font-size: 0.72rem;
      font-weight: 600;
      border-radius: 6px;
      border: 1px solid var(--card-border);
      background: #090e16;
      color: var(--text-muted);
      cursor: pointer;
      transition: all 0.15s ease;
    }}

    .filter-btn.active {{
      background: #1e293b;
      color: #fff;
      border-color: var(--accent-cyan);
    }}

    .search-input {{
      padding: 6px 12px;
      background: #090e16;
      border: 1px solid var(--card-border);
      border-radius: 6px;
      color: #fff;
      font-size: 0.78rem;
      font-family: inherit;
      outline: none;
      width: 240px;
    }}

    .search-input:focus {{
      border-color: var(--accent-cyan);
    }}

    .table-container {{
      overflow-x: auto;
      max-height: 400px;
      border: 1px solid var(--card-border);
      border-radius: 8px;
    }}

    table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 0.8rem;
      text-align: left;
    }}

    thead {{
      position: sticky;
      top: 0;
      background: #0f1622;
      z-index: 10;
      box-shadow: 0 2px 4px rgba(0,0,0,0.3);
    }}

    th {{
      padding: 10px 14px;
      color: var(--text-muted);
      font-weight: 700;
      text-transform: uppercase;
      font-size: 0.7rem;
      letter-spacing: 0.6px;
      border-bottom: 1px solid var(--card-border);
      white-space: nowrap;
    }}

    td {{
      padding: 9px 14px;
      border-bottom: 1px solid rgba(255, 255, 255, 0.04);
      color: #cbd5e1;
      white-space: nowrap;
    }}

    tbody tr:hover {{
      background: rgba(255, 255, 255, 0.03);
    }}

    .dist-badge {{
      display: inline-flex;
      align-items: center;
      gap: 4px;
      padding: 3px 8px;
      border-radius: 4px;
      font-weight: 700;
      font-family: var(--font-mono);
      font-size: 0.75rem;
    }}

    .dist-crit {{
      background: rgba(239, 68, 68, 0.18);
      color: #f87171;
      border: 1px solid rgba(239, 68, 68, 0.35);
    }}

    .dist-warn {{
      background: rgba(245, 158, 11, 0.18);
      color: #fbbf24;
      border: 1px solid rgba(245, 158, 11, 0.35);
    }}

    .dist-safe {{
      background: rgba(16, 185, 129, 0.18);
      color: #34d399;
      border: 1px solid rgba(16, 185, 129, 0.35);
    }}

    .tag-pill {{
      display: inline-flex;
      align-items: center;
      gap: 5px;
      font-weight: 600;
    }}

    .action-badge {{
      font-size: 0.72rem;
      padding: 2px 8px;
      border-radius: 4px;
      font-family: var(--font-mono);
      font-weight: 700;
    }}

    .action-halt {{
      background: rgba(239, 68, 68, 0.15);
      color: #f87171;
      border: 1px solid rgba(239, 68, 68, 0.3);
    }}

    .action-avoid {{
      background: rgba(56, 189, 248, 0.15);
      color: #38bdf8;
      border: 1px solid rgba(56, 189, 248, 0.3);
    }}

    footer {{
      border-top: 1px solid var(--card-border);
      padding: 10px 24px;
      font-size: 0.72rem;
      color: var(--text-muted);
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-family: var(--font-mono);
      background: #090e16;
    }}
  </style>
</head>
<body>

  <!-- TOP HEADER BAR -->
  <header>
    <div class="brand">
      <div class="brand-icon">🛡️</div>
      <div class="brand-title">
        <h1>AUTONOMOUS PERIMETER ROVER</h1>
        <p>Mission Command & Security Teleoperation Console</p>
      </div>
    </div>

    <!-- ACTIVE MODE TOGGLE -->
    <div class="mode-switch-container">
      <button id="btnAutoMode" class="mode-btn active-auto" onclick="setControlMode('autonomous')">
        <span>🤖</span>
        <span>AUTONOMOUS PATROL</span>
      </button>
      <button id="btnManualMode" class="mode-btn" onclick="setControlMode('manual')">
        <span>🕹️</span>
        <span>MANUAL JOYSTICK</span>
      </button>
    </div>

    <!-- EMERGENCY STOP BUTTON -->
    <button id="btnEStop" class="estop-btn" onclick="toggleEmergencyStop()">
      <span>🛑</span>
      <span id="estopBtnText">EMERGENCY STOP</span>
    </button>

    <!-- LIVE CLOCK & STATUS -->
    <div class="sys-status">
      <div class="status-pill">
        <span class="pulse-dot"></span>
        <span id="sysStatusText">ROS 2 JAZZY ONLINE</span>
      </div>
      <span id="liveClock">--:--:--</span>
    </div>
  </header>

  <main>
    <!-- REAL-TIME LOCAL TELEOPERATION BANNER -->
    <div style="background: rgba(14, 165, 233, 0.12); border: 1px solid rgba(0, 240, 255, 0.35); padding: 10px 18px; border-radius: 8px; margin-bottom: 16px; display: flex; align-items: center; justify-content: space-between; font-size: 0.82rem; flex-wrap: wrap; gap: 8px;">
      <div style="display: flex; align-items: center; gap: 10px;">
        <span style="font-size: 1.2rem;">📡</span>
        <span><strong>Live Gazebo Teleoperation:</strong> To view the direct 30 FPS camera feed and drive the rover in your local Gazebo simulator, open <a href="http://localhost:5000" target="_blank" style="color: var(--accent-cyan); text-decoration: underline; font-weight: 700;">http://localhost:5000</a>. On Streamlit Cloud, you are viewing the live geofence audit logs & mission analytics.</span>
      </div>
      <a href="http://localhost:5000" target="_blank" style="background: var(--accent-cyan); color: #0a0e14; padding: 6px 14px; border-radius: 4px; font-weight: 700; font-size: 0.75rem; text-decoration: none; display: inline-flex; align-items: center; gap: 4px;">OPEN LOCALHOST:5000 ↗</a>
    </div>

    <!-- TOP DECK: CAMERA FEED + TELEMETRY GAUGES + JOYSTICK -->
    <div class="top-deck-grid">
      
      <!-- 1. LIVE SURVEILLANCE FEED -->
      <div class="card">
        <div class="card-header">
          <div class="card-title">
            <span>📹</span>
            <span>Live Surveillance Stream</span>
          </div>
          <span class="card-badge" id="cameraFpsBadge">30 FPS | MJPEG</span>
        </div>
        <div class="video-container">
          <!-- Real Gazebo Camera Stream (loads when viewing locally or when port 5000 is open) -->
          <img id="videoStream" src="http://localhost:5000/video_feed" alt="Rover Live Stream" style="display:none; width: 100%; height: 100%; object-fit: cover;" onload="this.style.display='block'; var vh=document.getElementById('vectorHud'); if(vh) vh.style.display='none';" onerror="this.style.display='none'; var vh=document.getElementById('vectorHud'); if(vh) vh.style.display='block';">
          <div id="vectorHud" style="width: 100%; height: 100%;">
          <!-- Vector Tactical Camera Stream HUD matching Picture 2 -->
          <svg viewBox="0 0 640 480" width="100%" height="100%" xmlns="http://www.w3.org/2000/svg">
            <!-- Simulated Outdoor Yard Compound -->
            <rect width="640" height="480" fill="#2d333b"/>
            <!-- Horizon & Sky -->
            <rect width="640" height="150" fill="#1b2028"/>
            <!-- Ground Floor -->
            <polygon points="0,150 640,150 640,480 0,480" fill="#3e4450"/>
            <!-- Containers in Distance -->
            <rect x="70" y="90" width="140" height="90" fill="#1e3a8a" stroke="#3b82f6" stroke-width="2"/>
            <rect x="230" y="90" width="140" height="90" fill="#991b1b" stroke="#ef4444" stroke-width="2"/>
            <!-- Trapezoid Geofence Zone Overlay -->
            <polygon points="100,165 540,165 615,445 25,445" fill="rgba(0, 240, 255, 0.16)" stroke="#00f0ff" stroke-width="3"/>
            
            <!-- Top HUD Banner -->
            <rect x="0" y="0" width="640" height="34" fill="rgba(15, 23, 42, 0.92)"/>
            <text x="14" y="23" fill="#00ff88" font-family="monospace" font-size="14" font-weight="bold">
              STATUS: OUTDOOR YARD CLEAR (GEOFENCE SECURE)
            </text>

            <!-- Tactical Compass Heading Indicator (Top Right) -->
            <rect x="500" y="44" width="125" height="115" rx="8" fill="rgba(15, 22, 32, 0.88)" stroke="#334155" stroke-width="1.5"/>
            <text x="515" y="62" fill="#94a3b8" font-family="monospace" font-size="9" font-weight="bold">ROVER HEADING</text>
            <circle cx="562" cy="100" r="26" fill="none" stroke="#475569" stroke-width="1.5"/>
            <text x="559" y="81" fill="#00f0ff" font-family="monospace" font-size="9" font-weight="bold">N</text>
            <text x="583" y="103" fill="#64748b" font-family="monospace" font-size="8">E</text>
            <text x="559" y="123" fill="#64748b" font-family="monospace" font-size="8">S</text>
            <text x="540" y="103" fill="#64748b" font-family="monospace" font-size="8">W</text>
            <circle cx="562" cy="100" r="8" fill="none" stroke="#00f0ff" stroke-width="2"/>
            <circle cx="562" cy="100" r="3" fill="#00ff88"/>
            <!-- Compass Heading Green Arrow -->
            <line x1="562" y1="100" x2="578" y2="114" stroke="#00ff88" stroke-width="3" stroke-linecap="round"/>
            <polygon points="584,118 574,118 580,108" fill="#00ff88"/>
            <text x="532" y="148" fill="#00ff88" font-family="monospace" font-size="11" font-weight="bold">-49° [SE]</text>

            <!-- Dynamic Movement Ribbon Badge (Bottom HUD) -->
            <rect x="180" y="414" width="280" height="26" rx="4" fill="rgba(15, 22, 32, 0.90)" stroke="#334155" stroke-width="1"/>
            <line x1="195" y1="427" x2="215" y2="427" stroke="#00ff88" stroke-width="2.5"/>
            <polygon points="219,427 212,423 212,431" fill="#00ff88"/>
            <text x="225" y="431" fill="#00ff88" font-family="monospace" font-size="10" font-weight="bold">
              DIR: SE (-49°) | PATROL ADVANCING
            </text>

            <!-- Bottom Ribbon -->
            <rect x="0" y="456" width="640" height="24" fill="#090e16"/>
            <text x="12" y="472" fill="#94a3b8" font-family="monospace" font-size="10">
              Surveillance Active | Press 'R' to Resume Patrol
            </text>
          </svg>
          </div>

          <div class="video-overlay-hud">
            <span class="hud-chip">🔴 LIVE</span>
            <span class="hud-chip" id="hudModeChip">AUTO</span>
            <span class="hud-chip" id="hudPoseChip">X: 1.2 Y: -0.7</span>
          </div>
        </div>

        <div class="checkpoint-tracker">
          <div class="wp-pill" id="pillWP1">WP1</div>
          <div class="wp-pill" id="pillWP2">WP2</div>
          <div class="wp-pill" id="pillWP3">WP3</div>
          <div class="wp-pill" id="pillWP4">WP4</div>
          <div class="wp-pill" id="pillWP5">WP5</div>
          <div class="wp-pill active" id="pillWP6">WP6</div>
        </div>
      </div>

      <!-- 2. REAL-TIME VELOCITY & TELEMETRY DASHBOARD -->
      <div class="card">
        <div class="card-header">
          <div class="card-title">
            <span>⚡</span>
            <span>Rover Kinematics & Velocity Telemetry</span>
          </div>
          <span class="card-badge" id="telemetryRateBadge">4 Hz Telemetry</span>
        </div>

        <div class="telemetry-deck">
          <!-- Primary Rover Linear Speed -->
          <div class="telemetry-card">
            <div class="telemetry-label">
              <span>Primary Rover Linear Speed</span>
              <span>v</span>
            </div>
            <div class="telemetry-value-box">
              <span class="telemetry-num" id="valPrimarySpeed">0.06</span>
              <span class="telemetry-unit">m/s</span>
            </div>
            <div class="telemetry-bar-wrap">
              <div class="telemetry-bar-fill bar-cyan" id="barPrimarySpeed" style="width: 24%;"></div>
            </div>
            <div class="telemetry-subtext" id="subPrimarySpeed">Forward Velocity (Max 0.26 m/s)</div>
          </div>

          <!-- Primary Rover Angular Velocity -->
          <div class="telemetry-card">
            <div class="telemetry-label">
              <span>Primary Angular Velocity</span>
              <span>&omega;</span>
            </div>
            <div class="telemetry-value-box">
              <span class="telemetry-num" id="valPrimaryAng">+0.05</span>
              <span class="telemetry-unit">rad/s</span>
            </div>
            <div class="telemetry-bar-wrap">
              <div class="telemetry-bar-fill bar-green" id="barPrimaryAng" style="width: 12%;"></div>
            </div>
            <div class="telemetry-subtext" id="subPrimaryAng">Rotation: 2.9°/s (CCW ↺)</div>
          </div>

          <!-- Secondary Rover Velocity -->
          <div class="telemetry-card">
            <div class="telemetry-label">
              <span>Secondary Rover Speed</span>
              <span>v<sub>2</sub></span>
            </div>
            <div class="telemetry-value-box">
              <span class="telemetry-num" id="valSecondarySpeed">0.25</span>
              <span class="telemetry-unit">m/s</span>
            </div>
            <div class="telemetry-bar-wrap">
              <div class="telemetry-bar-fill bar-yellow" id="barSecondarySpeed" style="width: 50%;"></div>
            </div>
            <div class="telemetry-subtext" id="subSecondaryDist">Range to Primary: 4.80m (Dynamic: No)</div>
          </div>

          <!-- Position & Compass Heading -->
          <div class="telemetry-card">
            <div class="telemetry-label">
              <span>Perimeter Heading & Pose</span>
              <span>&theta;</span>
            </div>
            <div class="telemetry-value-box">
              <span class="telemetry-num" id="valHeadingDeg">-49&deg;</span>
              <span class="telemetry-unit" id="valCardinal">[SE]</span>
            </div>
            <div class="telemetry-bar-wrap">
              <div class="telemetry-bar-fill bar-blue" id="barHeading" style="width: 45%;"></div>
            </div>
            <div class="telemetry-subtext" id="subPose">Pose: (x=1.22, y=-0.67)</div>
          </div>
        </div>

        <div class="quick-actions">
          <button class="btn-action btn-origin" style="background: rgba(0, 240, 255, 0.18); border-color: var(--accent-cyan); color: #00f0ff; font-weight: 700;" onclick="triggerResetToOrigin()">
            <span>🏠</span>
            <span>Reset to Origin & Patrol</span>
          </button>
          <button class="btn-action btn-resume" onclick="triggerResumePatrol()">
            <span>🔓</span>
            <span>Resume Patrol ('R')</span>
          </button>
          <button class="btn-action" onclick="triggerClearCostmaps()">
            <span>🧹</span>
            <span>Clear Costmaps</span>
          </button>
        </div>
      </div>

      <!-- 3. MANUAL JOYSTICK CONTROL DECK -->
      <div class="card">
        <div class="card-header">
          <div class="card-title">
            <span>🕹️</span>
            <span>Manual Joystick Teleop</span>
          </div>
          <span class="card-badge" id="joystickStatusBadge">Disabled (Auto)</span>
        </div>

        <div class="joystick-panel">
          <!-- Overlay lock when in Autonomous Mode -->
          <div id="joystickLock" class="joystick-overlay-lock">
            <span style="font-size: 2.2rem;">🔒</span>
            <p style="font-size: 0.85rem; font-weight: 700; color: #fff;">Autonomous Patrol Active</p>
            <p style="font-size: 0.72rem; color: var(--text-muted); max-width: 200px;">Switch mode above to enable manual joystick teleoperation.</p>
            <button class="btn-action" style="background: var(--accent-yellow); color: #000; font-weight: 700; border: none; padding: 7px 18px;" onclick="setControlMode('manual')">
              Enable Manual Mode
            </button>
          </div>

          <!-- Interactive Analog Joystick Canvas -->
          <div class="joystick-wrapper" id="joystickWrapper">
            <canvas id="joystickCanvas" width="200" height="200"></canvas>
          </div>

          <div class="joystick-readout">
            <span id="joyReadoutLin">LIN: +0.00 m/s</span>
            <span id="joyReadoutAng">ANG: +0.00 rad/s</span>
          </div>
          <div style="font-size: 0.68rem; color: var(--text-muted); font-family: var(--font-mono); margin-top: 6px;">
            Touch/Drag Joystick or press [W] [A] [S] [D]
          </div>
        </div>
      </div>

    </div>

    <!-- BOTTOM PANEL: REAL-TIME DETECTION EVENT LOG TABLE -->
    <div class="card table-card">
      <div class="card-header">
        <div class="card-title">
          <span>🚨</span>
          <span>Perimeter Detection & Intruder Log</span>
          <span class="status-pill" style="font-size: 0.68rem; padding: 2px 7px;">
            <span class="pulse-dot"></span>
            <span id="detectionLiveTag">ACTIVE 1 HZ STREAM</span>
          </span>
        </div>
        <div style="display: flex; gap: 8px;">
          <button class="btn-action" style="padding: 4px 10px; font-size: 0.72rem;" onclick="exportTableToCSV()">
            <span>📥</span>
            <span>Export CSV</span>
          </button>
          <span class="card-badge" id="totalEventsBadge">0 Events Logged</span>
        </div>
      </div>

      <div class="table-toolbar">
        <div class="filter-pills">
          <button class="filter-btn active" onclick="setTableFilter('ALL')">All Targets</button>
          <button class="filter-btn" onclick="setTableFilter('Human')">🚨 Human Intruders</button>
          <button class="filter-btn" onclick="setTableFilter('Rover')">🚗 Secondary Rovers</button>
          <button class="filter-btn" onclick="setTableFilter('Obstacle')">📦 Static Shapes</button>
        </div>
        <input type="text" id="tableSearch" class="search-input" placeholder="Search date, time, target..." oninput="handleTableSearch()">
      </div>

      <div class="table-container">
        <table id="detectionsTable">
          <thead>
            <tr>
              <th>#</th>
              <th>Date</th>
              <th>Time</th>
              <th>Target Spotted</th>
              <th>Distance to Rover</th>
              <th>Target Location (X, Y)</th>
              <th>Rover Position</th>
              <th>Confidence</th>
              <th>Protocol Action</th>
            </tr>
          </thead>
          <tbody id="detectionsTbody">
            <tr>
              <td colspan="9" style="text-align: center; color: var(--text-muted); padding: 24px;">
                Connecting to event database and waiting for perimeter breaches...
              </td>
            </tr>
          </tbody>
        </table>
      </div>
    </div>
  </main>

  <footer>
    <span>AUTONOMOUS PERIMETER ROVER &bull; GEOFENCE SENTINEL SYSTEM</span>
    <span id="footerConnectionInfo">Connected: Mission Command &bull; ROS 2 Jazzy &bull; Nav2 MPPI</span>
  </footer>

  <script>
    // --- EMBEDDED EVENTS FROM SIMULATION ---
    const embeddedDetections = {json_detections_str};

    // --- GLOBAL STATE ---
    let currentMode = "autonomous";
    let isEmergencyStop = false;
    let activeFilter = "ALL";
    let searchQuery = "";
    let detectionEvents = Array.isArray(embeddedDetections) && embeddedDetections.length > 0 ? embeddedDetections.slice().reverse() : [];

    // Telemetry state
    let simLinear = 0.06;
    let simAngular = 0.05;
    let simX = 1.22;
    let simY = -0.67;
    let simYawDeg = -49;
    let activeWpNum = 6;

    // Joystick Math & Teleop State
    let isDragging = false;
    let joyX = 0, joyY = 0;
    const JOY_RADIUS = 75;
    const KNOB_RADIUS = 28;
    const MAX_LINEAR_SPEED = 0.25;   // m/s
    const MAX_ANGULAR_SPEED = 1.0;   // rad/s

    // --- DOM REFERENCES ---
    const btnAuto = document.getElementById("btnAutoMode");
    const btnManual = document.getElementById("btnManualMode");
    const btnEStop = document.getElementById("btnEStop");
    const estopBtnText = document.getElementById("estopBtnText");
    const joystickLock = document.getElementById("joystickLock");
    const joystickStatusBadge = document.getElementById("joystickStatusBadge");
    const joyCanvas = document.getElementById("joystickCanvas");
    const joyCtx = joyCanvas.getContext("2d");
    const joyReadoutLin = document.getElementById("joyReadoutLin");
    const joyReadoutAng = document.getElementById("joyReadoutAng");

    // Telemetry DOM
    const valPrimarySpeed = document.getElementById("valPrimarySpeed");
    const barPrimarySpeed = document.getElementById("barPrimarySpeed");
    const valPrimaryAng = document.getElementById("valPrimaryAng");
    const barPrimaryAng = document.getElementById("barPrimaryAng");
    const subPrimaryAng = document.getElementById("subPrimaryAng");
    const valSecondarySpeed = document.getElementById("valSecondarySpeed");
    const barSecondarySpeed = document.getElementById("barSecondarySpeed");
    const subSecondaryDist = document.getElementById("subSecondaryDist");
    const valHeadingDeg = document.getElementById("valHeadingDeg");
    const valCardinal = document.getElementById("valCardinal");
    const subPose = document.getElementById("subPose");
    const hudModeChip = document.getElementById("hudModeChip");
    const hudPoseChip = document.getElementById("hudPoseChip");
    const totalEventsBadge = document.getElementById("totalEventsBadge");
    const detectionsTbody = document.getElementById("detectionsTbody");

    // --- LIVE CLOCK ---
    setInterval(() => {{
      const now = new Date();
      document.getElementById("liveClock").innerText = now.toTimeString().split(" ")[0];
    }}, 1000);

    // --- TOAST HUD NOTIFICATIONS ---
    function showToast(message, type = "info") {{
      let container = document.getElementById("toastContainer");
      if (!container) {{
        container = document.createElement("div");
        container.id = "toastContainer";
        container.style.cssText = "position: fixed; bottom: 24px; right: 24px; z-index: 9999; display: flex; flex-direction: column; gap: 10px;";
        document.body.appendChild(container);
      }}
      const toast = document.createElement("div");
      const colors = {{
        success: "border-color: #00ff88; background: rgba(5, 150, 105, 0.95); color: #fff;",
        warn: "border-color: #ffb700; background: rgba(217, 119, 6, 0.95); color: #fff;",
        error: "border-color: #ff3366; background: rgba(220, 38, 38, 0.95); color: #fff;",
        info: "border-color: #00f0ff; background: rgba(15, 23, 42, 0.95); color: #f1f5f9;"
      }};
      toast.style.cssText = `padding: 10px 18px; border-radius: 8px; border: 1.5px solid; font-size: 0.82rem; font-weight: 600; font-family: var(--font-mono); box-shadow: 0 4px 14px rgba(0,0,0,0.5); transition: opacity 0.3s ease; ${{colors[type] || colors.info}}`;
      toast.innerText = message;
      container.appendChild(toast);
      setTimeout(() => {{
        toast.style.opacity = "0";
        setTimeout(() => toast.remove(), 300);
      }}, 3500);
    }}

    // --- MODE SWITCHING ---
    function setControlMode(mode) {{
      if (isEmergencyStop) {{
        showToast("⚠️ Emergency Stop is active! Click 'Reset to Origin & Patrol' first.", "warn");
        return;
      }}
      currentMode = mode;
      if (mode === "autonomous") {{
        btnAuto.className = "mode-btn active-auto";
        btnManual.className = "mode-btn";
        joystickLock.classList.remove("hidden");
        joystickStatusBadge.innerText = "Disabled (Auto)";
        hudModeChip.innerText = "AUTO";
        showToast("🤖 Autonomous Patrol Active - Resuming perimeter navigation...", "success");
      }} else {{
        btnAuto.className = "mode-btn";
        btnManual.className = "mode-btn active-manual";
        joystickLock.classList.add("hidden");
        joystickStatusBadge.innerText = "Active (Teleop)";
        hudModeChip.innerText = "MANUAL";
        showToast("🕹️ Manual Joystick Teleop Active. Use Joystick or [W][A][S][D].", "warn");
      }}

      fetch("/api/mode", {{
        method: "POST",
        headers: {{ "Content-Type": "application/json" }},
        body: JSON.stringify({{ mode: mode }})
      }}).catch(err => {{}});
      fetch("http://localhost:5000/api/mode", {{
        method: "POST",
        headers: {{ "Content-Type": "application/json" }},
        body: JSON.stringify({{ mode: mode }})
      }}).catch(err => {{}});
    }}

    // --- EMERGENCY STOP & RESET ---
    function toggleEmergencyStop() {{
      if (!isEmergencyStop) {{
        isEmergencyStop = true;
        btnEStop.classList.add("active-stopped");
        estopBtnText.innerText = "RESET TO ORIGIN & PATROL";
        joystickLock.classList.remove("hidden");
        joystickStatusBadge.innerText = "E-STOPPED";
        hudModeChip.innerText = "HALTED";
        simLinear = 0.00;
        simAngular = 0.00;
        updateTelemetryDisplay();
        showToast("🛑 EMERGENCY STOP TRIGGERED! Rover halted.", "error");

        fetch("/api/estop", {{
          method: "POST",
          headers: {{ "Content-Type": "application/json" }},
          body: JSON.stringify({{ stop: true }})
        }}).catch(err => {{}});
        fetch("http://localhost:5000/api/estop", {{
          method: "POST",
          headers: {{ "Content-Type": "application/json" }},
          body: JSON.stringify({{ stop: true }})
        }}).catch(err => {{}});
      }} else {{
        triggerResetToOrigin();
      }}
    }}

    // --- RESET TO ORIGIN & PATROL ROUTINE ---
    function triggerResetToOrigin() {{
      isEmergencyStop = false;
      btnEStop.classList.remove("active-stopped");
      estopBtnText.innerText = "EMERGENCY STOP";
      currentMode = "autonomous";
      btnAuto.className = "mode-btn active-auto";
      btnManual.className = "mode-btn";
      joystickLock.classList.remove("hidden");
      joystickStatusBadge.innerText = "Disabled (Auto)";
      hudModeChip.innerText = "AUTO";

      simX = 0.00;
      simY = -4.50;
      simYawDeg = 0;
      simLinear = 0.18;
      activeWpNum = 1;
      updateTelemetryDisplay();

      showToast("🔄 Rover reset to Origin (0.0, -4.50) & Autonomous Surveillance Patrol Started!", "success");

      fetch("/api/reset_to_origin", {{ method: "POST" }}).catch(err => {{}});
      fetch("http://localhost:5000/api/reset_to_origin", {{ method: "POST" }}).catch(err => {{}});
    }}

    // --- QUICK ACTIONS ---
    function triggerResumePatrol() {{
      showToast("🔓 Operator RESUME sent (35s bypass grace period armed)!", "info");
      fetch("/api/resume", {{ method: "POST" }}).catch(err => {{}});
    }}

    function triggerClearCostmaps() {{
      showToast("🧹 Nav2 Local & Global Costmaps cleared!", "info");
      fetch("/api/clear_costmaps", {{ method: "POST" }}).catch(err => {{}});
    }}

    // --- JOYSTICK IMPLEMENTATION ---
    const joyCenter = 100;

    function renderJoystick(knobX, knobY) {{
      joyCtx.clearRect(0, 0, joyCanvas.width, joyCanvas.height);

      // Outer boundary ring
      joyCtx.beginPath();
      joyCtx.arc(joyCenter, joyCenter, JOY_RADIUS, 0, Math.PI * 2);
      joyCtx.fillStyle = "rgba(18, 26, 38, 0.85)";
      joyCtx.fill();
      joyCtx.lineWidth = 2;
      joyCtx.strokeStyle = "rgba(0, 240, 255, 0.4)";
      joyCtx.stroke();

      // Crosshair lines
      joyCtx.beginPath();
      joyCtx.strokeStyle = "rgba(255, 255, 255, 0.1)";
      joyCtx.lineWidth = 1;
      joyCtx.moveTo(joyCenter - JOY_RADIUS, joyCenter);
      joyCtx.lineTo(joyCenter + JOY_RADIUS, joyCenter);
      joyCtx.moveTo(joyCenter, joyCenter - JOY_RADIUS);
      joyCtx.lineTo(joyCenter, joyCenter + JOY_RADIUS);
      joyCtx.stroke();

      // Cardinal labels
      joyCtx.font = "9px monospace";
      joyCtx.fillStyle = "rgba(255, 255, 255, 0.4)";
      joyCtx.textAlign = "center";
      joyCtx.fillText("▲ FWD", joyCenter, joyCenter - JOY_RADIUS + 14);
      joyCtx.fillText("▼ REV", joyCenter, joyCenter + JOY_RADIUS - 6);
      joyCtx.fillText("◀ L", joyCenter - JOY_RADIUS + 14, joyCenter + 3);
      joyCtx.fillText("R ▶", joyCenter + JOY_RADIUS - 14, joyCenter + 3);

      // Vector indicator line
      if (knobX !== joyCenter || knobY !== joyCenter) {{
        joyCtx.beginPath();
        joyCtx.moveTo(joyCenter, joyCenter);
        joyCtx.lineTo(knobX, knobY);
        joyCtx.strokeStyle = "rgba(0, 255, 136, 0.6)";
        joyCtx.lineWidth = 2;
        joyCtx.stroke();
      }}

      // Draggable thumb knob
      const knobGrad = joyCtx.createRadialGradient(knobX - 4, knobY - 4, 3, knobX, knobY, KNOB_RADIUS);
      knobGrad.addColorStop(0, "#00ff88");
      knobGrad.addColorStop(1, "#059669");

      joyCtx.beginPath();
      joyCtx.arc(knobX, knobY, KNOB_RADIUS, 0, Math.PI * 2);
      joyCtx.fillStyle = knobGrad;
      joyCtx.fill();
      joyCtx.lineWidth = 2;
      joyCtx.strokeStyle = "#ffffff";
      joyCtx.stroke();
    }}

    renderJoystick(joyCenter, joyCenter);

    function updateJoystickPosition(clientX, clientY) {{
      const rect = joyCanvas.getBoundingClientRect();
      let dx = clientX - (rect.left + joyCenter);
      let dy = clientY - (rect.top + joyCenter);
      const dist = Math.hypot(dx, dy);

      if (dist > JOY_RADIUS - KNOB_RADIUS + 6) {{
        const angle = Math.atan2(dy, dx);
        dx = Math.cos(angle) * (JOY_RADIUS - KNOB_RADIUS + 6);
        dy = Math.sin(angle) * (JOY_RADIUS - KNOB_RADIUS + 6);
      }}

      joyX = dx / (JOY_RADIUS - KNOB_RADIUS + 6);
      joyY = dy / (JOY_RADIUS - KNOB_RADIUS + 6);

      renderJoystick(joyCenter + dx, joyCenter + dy);

      const linearCmd = -joyY * MAX_LINEAR_SPEED;
      const angularCmd = -joyX * MAX_ANGULAR_SPEED;

      joyReadoutLin.innerText = `LIN: ${{ (linearCmd >= 0 ? "+" : "") + linearCmd.toFixed(2) }} m/s`;
      joyReadoutAng.innerText = `ANG: ${{ (angularCmd >= 0 ? "+" : "") + angularCmd.toFixed(2) }} rad/s`;

      if (currentMode === "manual" && !isEmergencyStop) {{
        simLinear = Math.abs(linearCmd);
        simAngular = angularCmd;
        simYawDeg = (simYawDeg + angularCmd * 4) % 360;
        updateTelemetryDisplay();
      }}

      sendTeleopCmd(linearCmd, angularCmd);
    }}

    function sendTeleopCmd(linear, angular) {{
      if (currentMode !== "manual" || isEmergencyStop) return;
      fetch("/api/teleop", {{
        method: "POST",
        headers: {{ "Content-Type": "application/json" }},
        body: JSON.stringify({{ linear: linear, angular: angular }})
      }}).catch(err => {{}});
      // Also bridge directly to local Gazebo controller if open locally
      fetch("http://localhost:5000/api/teleop", {{
        method: "POST",
        headers: {{ "Content-Type": "application/json" }},
        body: JSON.stringify({{ linear: linear, angular: angular }})
      }}).catch(err => {{}});
    }}

    // Pointer events on joystick
    joyCanvas.addEventListener("pointerdown", e => {{
      if (currentMode !== "manual" || isEmergencyStop) return;
      isDragging = true;
      joyCanvas.setPointerCapture(e.pointerId);
      updateJoystickPosition(e.clientX, e.clientY);
    }});

    joyCanvas.addEventListener("pointermove", e => {{
      if (!isDragging) return;
      updateJoystickPosition(e.clientX, e.clientY);
    }});

    function resetJoystick() {{
      if (!isDragging) return;
      isDragging = false;
      joyX = 0; joyY = 0;
      renderJoystick(joyCenter, joyCenter);
      joyReadoutLin.innerText = "LIN: +0.00 m/s";
      joyReadoutAng.innerText = "ANG: +0.00 rad/s";
      if (currentMode === "manual") {{
        simLinear = 0.00;
        simAngular = 0.00;
        updateTelemetryDisplay();
      }}
      sendTeleopCmd(0.0, 0.0);
    }}

    joyCanvas.addEventListener("pointerup", resetJoystick);
    joyCanvas.addEventListener("pointercancel", resetJoystick);

    // Keyboard WASD Controls
    const keysPressed = {{}};
    window.addEventListener("keydown", e => {{
      if (currentMode !== "manual" || isEmergencyStop) return;
      const k = e.key.toLowerCase();
      if (["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright"].includes(k)) {{
        keysPressed[k] = true;
        handleKeyboardTeleop();
      }}
    }});

    window.addEventListener("keyup", e => {{
      const k = e.key.toLowerCase();
      if (keysPressed[k]) {{
        delete keysPressed[k];
        handleKeyboardTeleop();
      }}
    }});

    function handleKeyboardTeleop() {{
      if (currentMode !== "manual" || isEmergencyStop) return;
      let lin = 0.0;
      let ang = 0.0;
      if (keysPressed["w"] || keysPressed["arrowup"]) lin += MAX_LINEAR_SPEED;
      if (keysPressed["s"] || keysPressed["arrowdown"]) lin -= MAX_LINEAR_SPEED * 0.8;
      if (keysPressed["a"] || keysPressed["arrowleft"]) ang += MAX_ANGULAR_SPEED * 0.8;
      if (keysPressed["d"] || keysPressed["arrowright"]) ang -= MAX_ANGULAR_SPEED * 0.8;

      const normY = -lin / MAX_LINEAR_SPEED;
      const normX = -ang / MAX_ANGULAR_SPEED;
      renderJoystick(joyCenter + normX * 45, joyCenter + normY * 45);

      joyReadoutLin.innerText = `LIN: ${{ (lin >= 0 ? "+" : "") + lin.toFixed(2) }} m/s`;
      joyReadoutAng.innerText = `ANG: ${{ (ang >= 0 ? "+" : "") + ang.toFixed(2) }} rad/s`;

      simLinear = Math.abs(lin);
      simAngular = ang;
      simYawDeg = (simYawDeg + ang * 3) % 360;
      updateTelemetryDisplay();

      sendTeleopCmd(lin, ang);
    }}

    // --- TELEMETRY RENDER HELPER ---
    function updateTelemetryDisplay() {{
      valPrimarySpeed.innerText = simLinear.toFixed(2);
      barPrimarySpeed.style.width = Math.min((simLinear / 0.28) * 100, 100) + "%";

      valPrimaryAng.innerText = (simAngular >= 0 ? "+" : "") + simAngular.toFixed(2);
      barPrimaryAng.style.width = Math.min((Math.abs(simAngular) / 1.2) * 100, 100) + "%";
      const degSec = (simAngular * 180 / Math.PI).toFixed(1);
      subPrimaryAng.innerText = `Rotation: ${{degSec}}°/s ${{ simAngular > 0.02 ? '(CCW ↺)' : (simAngular < -0.02 ? '(CW ↻)' : '') }}`;

      valHeadingDeg.innerHTML = `${{ simYawDeg >= 0 ? '+' : '' }}${{ simYawDeg.toFixed(0) }}&deg;`;
      subPose.innerText = `Pose: (x=${{simX.toFixed(2)}}, y=${{simY.toFixed(2)}})`;
      hudPoseChip.innerText = `X: ${{simX.toFixed(1)}} Y: ${{simY.toFixed(1)}}`;

      for (let i = 1; i <= 6; i++) {{
        const pill = document.getElementById("pillWP" + i);
        if (pill) {{
          pill.className = (i === activeWpNum) ? "wp-pill active" : "wp-pill";
        }}
      }}
    }}

    // --- TELEMETRY POLLING WITH FALLBACK SIMULATION ---
    function fetchTelemetry() {{
      fetch("/api/telemetry")
        .then(res => res.json())
        .then(data => {{
          if (!data) return;
          simLinear = data.primary_rover.linear_velocity || 0.0;
          simAngular = data.primary_rover.angular_velocity || 0.0;
          simX = data.primary_rover.x || 0.0;
          simY = data.primary_rover.y || -4.5;
          simYawDeg = data.primary_rover.yaw_deg || 0;
          activeWpNum = data.active_waypoint || 1;
          updateTelemetryDisplay();
        }})
        .catch(() => {{
          // Offline Simulation (Smoothly patrol perimeter if in Auto mode)
          if (currentMode === "autonomous" && !isEmergencyStop) {{
            simLinear = 0.06 + Math.sin(Date.now() / 800) * 0.03;
            simAngular = Math.sin(Date.now() / 1500) * 0.08;
            simYawDeg = -49 + Math.sin(Date.now() / 2000) * 12;
            updateTelemetryDisplay();
          }}
        }});
    }}
    setInterval(fetchTelemetry, 250);

    // --- DETECTIONS LOG RENDERING ---
    function setTableFilter(filter) {{
      activeFilter = filter;
      document.querySelectorAll(".filter-btn").forEach(btn => btn.classList.remove("active"));
      event.target.classList.add("active");
      renderTable();
    }}

    function handleTableSearch() {{
      searchQuery = document.getElementById("tableSearch").value.toLowerCase();
      renderTable();
    }}

    function renderTable() {{
      totalEventsBadge.innerText = `${{detectionEvents.length}} Events Logged`;
      let filtered = detectionEvents;

      if (activeFilter === "Human") {{
        filtered = filtered.filter(e => (e.target || "").toLowerCase().includes("human"));
      }} else if (activeFilter === "Rover") {{
        filtered = filtered.filter(e => (e.target || "").toLowerCase().includes("rover"));
      }} else if (activeFilter === "Obstacle") {{
        filtered = filtered.filter(e => !(e.target || "").toLowerCase().includes("human") && !(e.target || "").toLowerCase().includes("rover"));
      }}

      if (searchQuery) {{
        filtered = filtered.filter(e => 
          (e.date || "").toLowerCase().includes(searchQuery) ||
          (e.time || "").toLowerCase().includes(searchQuery) ||
          (e.target || "").toLowerCase().includes(searchQuery) ||
          (e.status || "").toLowerCase().includes(searchQuery)
        );
      }}

      if (filtered.length === 0) {{
        detectionsTbody.innerHTML = `<tr><td colspan="9" style="text-align: center; color: var(--text-muted); padding: 20px;">No detection events match the current filter.</td></tr>`;
        return;
      }}

      let html = "";
      for (let i = 0; i < Math.min(filtered.length, 120); i++) {{
        const ev = filtered[i];
        const dist = parseFloat(ev.distance_meters) || 0.0;
        let distClass = "dist-safe";
        if (dist < 2.5) distClass = "dist-crit";
        else if (dist < 4.5) distClass = "dist-warn";

        const isHuman = (ev.target || "").includes("Human");
        const isRover = (ev.target || "").includes("Rover");
        const icon = isHuman ? "🚨" : (isRover ? "🚗" : "📦");

        let actionBadge = `<span class="action-badge action-halt">HALTED & HELD</span>`;
        if (isRover) {{
          actionBadge = `<span class="action-badge action-avoid">AVOID & YIELD</span>`;
        }}

        const conf = ev.confidence ? (ev.confidence * 100).toFixed(0) + "%" : "95%";
        const targetCoords = ev.target_coordinates ? `(${{ev.target_coordinates.x.toFixed(2)}}, ${{ev.target_coordinates.y.toFixed(2)}})` : "N/A";
        const roverPos = ev.rover_position ? `(${{ev.rover_position.x.toFixed(2)}}, ${{ev.rover_position.y.toFixed(2)}})` : "N/A";

        html += `
          <tr>
            <td style="font-family: var(--font-mono); color: var(--accent-cyan); font-weight: bold;">#${{ev.event_id || (i + 1)}}</td>
            <td style="font-family: var(--font-mono);">${{ev.date || '2026-11-16'}}</td>
            <td style="font-family: var(--font-mono); font-weight: 600; color: #fff;">${{ev.time || '12:32:00'}}</td>
            <td><span class="tag-pill">${{icon}} <strong>${{ev.target || 'Obstacle'}}</strong></span></td>
            <td><span class="dist-badge ${{distClass}}">${{dist.toFixed(2)}} m</span></td>
            <td style="font-family: var(--font-mono); font-size: 0.75rem;">${{targetCoords}}</td>
            <td style="font-family: var(--font-mono); font-size: 0.75rem;">${{roverPos}}</td>
            <td><span style="font-family: var(--font-mono); color: var(--accent-green);">${{conf}}</span></td>
            <td>${{actionBadge}}</td>
          </tr>
        `;
      }}
      detectionsTbody.innerHTML = html;
    }}

    // Export CSV
    function exportTableToCSV() {{
      if (detectionEvents.length === 0) {{
        alert("No detection events to export!");
        return;
      }}
      let csv = "EventID,Date,Time,Target,Distance_Meters,Target_X,Target_Y,Rover_X,Rover_Y,Confidence,Status\\n";
      detectionEvents.forEach(e => {{
        const tx = e.target_coordinates ? e.target_coordinates.x : "";
        const ty = e.target_coordinates ? e.target_coordinates.y : "";
        const rx = e.rover_position ? e.rover_position.x : "";
        const ry = e.rover_position ? e.rover_position.y : "";
        csv += `${{e.event_id}},"${{e.date}}","${{e.time}}","${{e.target}}",${{e.distance_meters}},${{tx}},${{ty}},${{rx}},${{ry}},${{e.confidence}},"${{e.status}}"\\n`;
      }});
      const blob = new Blob([csv], {{ type: "text/csv;charset=utf-8;" }});
      const url = URL.createObjectURL(blob);
      const link = document.createElement("a");
      link.setAttribute("href", url);
      link.setAttribute("download", `perimeter_detections_${{new Date().toISOString().slice(0, 10)}}.csv`);
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
    }}

    // Initial render
    renderTable();
    updateTelemetryDisplay();
  </script>
</body>
</html>"""

# Render the entire Mission Control GCS Dashboard component
components.html(DASHBOARD_HTML, height=1320, scrolling=True)
