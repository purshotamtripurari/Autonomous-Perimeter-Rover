#!/usr/bin/env python3
"""
Autonomous Perimeter Rover - Mission Command & Security Analytics Dashboard
Streamlit Cloud Web Application
Features:
- Real-time & Historical Intruder Detection Analytics
- Interactive 2D Yard Tactical Radar Map (Plotly) with Waypoint Progression
- Full Audit Log Table with Multi-Filter, Search & CSV/JSON Export
- Statistical Breakdown of Detection Range, Target Types & AI Confidence
- Mission Architecture & Surveillance Visual Evidence Gallery
"""

import json
import math
import os
from datetime import datetime

import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
import streamlit as st
from PIL import Image

# -----------------------------------------------------------------------------
# 1. PAGE CONFIGURATION & THEME STYLING
# -----------------------------------------------------------------------------
st.set_page_config(
    page_title="Autonomous Perimeter Rover - Security Command",
    page_icon="🛡️",
    layout="wide",
    initial_sidebar_state="expanded",
)

# Custom Tactical Dark Theme CSS
st.markdown(
    """
    <style>
    /* Global Base */
    .stApp {
        background-color: #0b1017;
        color: #f1f5f9;
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    }
    
    /* Metric Cards */
    div[data-testid="stMetric"] {
        background: linear-gradient(135deg, #131b26 0%, #1a2434 100%);
        border: 1px solid #233145;
        border-radius: 10px;
        padding: 14px 18px;
        box-shadow: 0 4px 12px rgba(0, 0, 0, 0.35);
    }
    div[data-testid="stMetricLabel"] p {
        color: #94a3b8 !important;
        font-size: 0.82rem !important;
        font-weight: 600 !important;
        text-transform: uppercase;
        letter-spacing: 0.5px;
    }
    div[data-testid="stMetricValue"] div {
        color: #00f0ff !important;
        font-size: 1.85rem !important;
        font-weight: 700 !important;
        font-family: ui-monospace, monospace;
    }
    
    /* Header & Badge Pills */
    .header-badge {
        display: inline-flex;
        align-items: center;
        gap: 6px;
        padding: 4px 12px;
        border-radius: 20px;
        font-size: 0.75rem;
        font-weight: 600;
        font-family: ui-monospace, monospace;
        background: rgba(0, 240, 255, 0.12);
        color: #00f0ff;
        border: 1px solid rgba(0, 240, 255, 0.3);
        margin-right: 8px;
    }
    .badge-live {
        background: rgba(16, 185, 129, 0.15);
        color: #34d399;
        border-color: rgba(16, 185, 129, 0.35);
    }
    .badge-alert {
        background: rgba(239, 68, 68, 0.15);
        color: #f87171;
        border-color: rgba(239, 68, 68, 0.35);
    }
    
    /* Section Dividers */
    hr {
        border-color: #1e293b;
        margin: 1.5rem 0;
    }
    </style>
    """,
    unsafe_allow_html=True,
)


# -----------------------------------------------------------------------------
# 2. DATA INGESTION & CACHED PREPROCESSING
# -----------------------------------------------------------------------------
DATA_PATH = os.path.join(os.path.dirname(__file__), "perimeter_detections.json")


def categorize_target(target_str: str) -> str:
    s = str(target_str).lower()
    if "human" in s or "person" in s:
        return "Human Intruder"
    elif "rover" in s or "secondary" in s:
        return "Secondary Rover"
    else:
        return "Foreign Obstacle"


@st.cache_data(ttl=60)
def load_detection_data():
    if not os.path.exists(DATA_PATH):
        # Fallback dummy sample if deployed without JSON
        return pd.DataFrame([
            {
                "event_id": 1,
                "timestamp": "2026-10-30 14:44:51",
                "date": "2026-10-30",
                "time": "14:44:51",
                "target": "Human Intruder",
                "category": "Human Intruder",
                "confidence": 0.88,
                "distance_meters": 2.45,
                "rover_x": 0.5,
                "rover_y": -4.5,
                "target_x": 3.6,
                "target_y": 0.3,
                "status": "DETECTED_AND_HALTED",
            }
        ])

    with open(DATA_PATH, "r") as f:
        try:
            raw_data = json.load(f)
        except Exception:
            raw_data = []

    records = []
    for item in raw_data:
        target = item.get("target", "Unknown Obstacle")
        rover_pos = item.get("rover_position", {}) or {}
        target_coords = item.get("target_coordinates", {}) or {}

        records.append({
            "event_id": item.get("event_id", len(records) + 1),
            "timestamp": item.get("timestamp", ""),
            "date": item.get("date", ""),
            "time": item.get("time", ""),
            "target": target,
            "category": categorize_target(target),
            "confidence": float(item.get("confidence", 0.90)),
            "distance_meters": float(item.get("distance_meters", 3.0)),
            "rover_x": float(rover_pos.get("x", 0.0)),
            "rover_y": float(rover_pos.get("y", 0.0)),
            "rover_yaw": float(rover_pos.get("yaw_degrees", 0.0)),
            "target_x": float(target_coords.get("x", 0.0)),
            "target_y": float(target_coords.get("y", 0.0)),
            "status": item.get("status", "DETECTED_AND_HALTED"),
        })

    df = pd.DataFrame(records)
    if not df.empty:
        df["parsed_time"] = pd.to_datetime(df["timestamp"], errors="coerce")
        df["hour"] = df["parsed_time"].dt.hour
    return df


df_all = load_detection_data()


# -----------------------------------------------------------------------------
# 3. SIDEBAR CONTROLS & FILTERS
# -----------------------------------------------------------------------------
st.sidebar.markdown(
    """
    <div style="text-align: center; margin-bottom: 20px;">
        <h2 style="margin: 0; color: #fff; font-size: 1.35rem;">🛡️ PERIMETER ROVER</h2>
        <p style="color: #00f0ff; font-size: 0.75rem; font-family: monospace; letter-spacing: 1px; margin-top: 4px;">
            MISSION COMMAND CONSOLE
        </p>
    </div>
    """,
    unsafe_allow_html=True,
)

st.sidebar.markdown("### 🎛️ Incident Filters")

# Filter: Target Categories
categories = sorted(df_all["category"].unique()) if not df_all.empty else ["Human Intruder", "Secondary Rover", "Foreign Obstacle"]
selected_categories = st.sidebar.multiselect(
    "Target Classification",
    options=categories,
    default=categories,
)

# Filter: Distance Hazard Range
max_dist = float(df_all["distance_meters"].max()) if not df_all.empty else 10.0
dist_range = st.sidebar.slider(
    "Hazard Distance Range (m)",
    min_value=0.0,
    max_value=max(10.0, max_dist),
    value=(0.0, 8.0),
    step=0.25,
)

# Filter: Confidence Threshold
conf_thresh = st.sidebar.slider(
    "Min AI Detection Confidence",
    min_value=0.50,
    max_value=0.99,
    value=0.70,
    step=0.05,
)

# Apply Filters
mask = (
    df_all["category"].isin(selected_categories)
    & (df_all["distance_meters"] >= dist_range[0])
    & (df_all["distance_meters"] <= dist_range[1])
    & (df_all["confidence"] >= conf_thresh)
)
df_filtered = df_all[mask].copy()

st.sidebar.markdown("---")
st.sidebar.markdown("### 📡 System Architecture")
st.sidebar.info(
    """
    - **OS**: Ubuntu 24.04 LTS
    - **Middleware**: ROS 2 Jazzy
    - **Navigation**: Nav2 MPPI
    - **Perception**: YOLOv8n + LiDAR
    - **Simulator**: Gazebo Harmonic
    - **Host**: TurtleBot3 Waffle
    """
)

st.sidebar.markdown("---")
st.sidebar.markdown(
    """
    <div style="font-size: 0.72rem; color: #64748b; font-family: monospace;">
        Autonomous Perimeter Surveillance System<br>
        Geofencing & Security Sentinel
    </div>
    """,
    unsafe_allow_html=True,
)


# -----------------------------------------------------------------------------
# 4. TOP TITLE & KPI METRICS
# -----------------------------------------------------------------------------
col_title, col_badges = st.columns([0.65, 0.35])
with col_title:
    st.markdown(
        """
        <h1 style="margin-bottom: 4px; font-size: 2.1rem; color: #fff;">
            Autonomous Perimeter Security Rover
        </h1>
        <p style="color: #94a3b8; font-size: 0.95rem; margin-top: 0;">
            Mission Command, Detection Geofencing & Intrusion Audit Dashboard
        </p>
        """,
        unsafe_allow_html=True,
    )

with col_badges:
    st.markdown(
        """
        <div style="text-align: right; margin-top: 10px;">
            <span class="header-badge badge-live">● SYSTEM ACTIVE</span>
            <span class="header-badge">ROS 2 JAZZY</span>
            <span class="header-badge">NAV2 MPPI</span>
            <span class="header-badge">YOLOV8 SENTINEL</span>
        </div>
        """,
        unsafe_allow_html=True,
    )

st.markdown("<hr style='margin: 0.8rem 0;'>", unsafe_allow_html=True)

# Top KPI Metric Cards
m1, m2, m3, m4, m5 = st.columns(5)
total_events = len(df_filtered)
human_breaches = len(df_filtered[df_filtered["category"] == "Human Intruder"])
rover_yields = len(df_filtered[df_filtered["category"] == "Secondary Rover"])
avg_dist = df_filtered["distance_meters"].mean() if total_events > 0 else 0.0
high_conf = len(df_filtered[df_filtered["confidence"] >= 0.85])

m1.metric("Total Events", f"{total_events}", f"Logged in Yard")
m2.metric("Human Intruders", f"{human_breaches}", f"Halt & Hold Protocol")
m3.metric("Secondary Rovers", f"{rover_yields}", f"Evasive Yield & Pass")
m4.metric("Avg Breach Range", f"{avg_dist:.2f} m", "Forward LiDAR / Vision")
m5.metric("High Confidence", f"{high_conf}", "≥ 85% Precision")


# -----------------------------------------------------------------------------
# 5. TABBED INTERACTIVE DECK
# -----------------------------------------------------------------------------
tab_map, tab_analytics, tab_table, tab_evidence, tab_arch = st.tabs([
    "🗺️ Tactical Radar Yard Map",
    "📊 Security Incident Analytics",
    "📋 Intrusion Audit Log",
    "📸 Visual Evidence Gallery",
    "⚙️ System Specifications",
])


# -----------------------------------------------------------------------------
# TAB 1: INTERACTIVE 2D TACTICAL YARD RADAR MAP
# -----------------------------------------------------------------------------
with tab_map:
    st.markdown("### 🗺️ Outdoor Security Yard Tactical Compound (10m × 10m)")
    st.caption("Live spatial plot showing perimeter fence boundaries, yard obstacles, Nav2 patrol checkpoints, and logged security breaches.")

    # Create 2D Tactical Radar Map with Plotly
    fig_map = go.Figure()

    # 1. Compound Perimeter Boundary Walls
    fig_map.add_trace(go.Scatter(
        x=[-4.8, 4.8, 4.8, -4.8, -4.8],
        y=[-5.8, -5.8, 5.8, 5.8, -5.8],
        mode="lines",
        line=dict(color="#38bdf8", width=3),
        name="Security Perimeter Wall",
        hoverinfo="skip"
    ))

    # 2. Central Shipping Containers & Guard Booth
    # Blue Container (North)
    fig_map.add_shape(type="rect", x0=-1.5, y0=0.8, x1=1.5, y1=2.8,
                      fillcolor="#1e3a8a", line=dict(color="#3b82f6", width=2), opacity=0.8)
    # Red Container (South)
    fig_map.add_shape(type="rect", x0=-1.5, y0=-2.2, x1=1.5, y1=-0.2,
                      fillcolor="#991b1b", line=dict(color="#ef4444", width=2), opacity=0.8)
    # Guard Booth (South-West Corner)
    fig_map.add_shape(type="rect", x0=-4.5, y0=-4.8, x1=-2.7, y1=-3.0,
                      fillcolor="#334155", line=dict(color="#64748b", width=2), opacity=0.8)
    # Central Yard Platform / Obstacle
    fig_map.add_shape(type="rect", x0=-0.6, y0=-3.8, x1=0.6, y1=-2.6,
                      fillcolor="#b45309", line=dict(color="#f59e0b", width=2), opacity=0.7)

    # 3. Perimeter Checkpoint Patrol Route (WP1 -> WP2 -> WP3 -> WP4 -> WP5 -> WP6 -> WP1)
    wp_coords = [
        (3.5, -4.5, "WP1 (South-East)"),
        (3.5, 4.5, "WP2 (North-East)"),
        (-3.5, 4.5, "WP3 (North-West)"),
        (-3.5, -1.5, "WP4 (West Corridor)"),
        (-2.0, -4.5, "WP5 (South-West)"),
        (0.0, -4.5, "WP6 (Origin Spawn)"),
    ]
    wx = [p[0] for p in wp_coords] + [wp_coords[0][0]]
    wy = [p[1] for p in wp_coords] + [wp_coords[0][1]]
    wnames = [p[2] for p in wp_coords] + [wp_coords[0][2]]

    fig_map.add_trace(go.Scatter(
        x=wx, y=wy,
        mode="lines+markers+text",
        line=dict(color="#00f0ff", width=2, dash="dash"),
        marker=dict(size=11, color="#00ff88", symbol="diamond"),
        text=[f"WP{i+1}" for i in range(len(wp_coords))] + [""],
        textposition="top center",
        textfont=dict(color="#00ff88", size=10, family="monospace"),
        name="Nav2 Patrol Loop",
        hovertext=wnames,
        hoverinfo="text"
    ))

    # 4. Security Detections Points Plotted by Category
    color_map = {
        "Human Intruder": "#ef4444",      # Vivid Red
        "Secondary Rover": "#38bdf8",     # Cyan
        "Foreign Obstacle": "#f59e0b",    # Amber
    }
    symbol_map = {
        "Human Intruder": "x",
        "Secondary Rover": "circle",
        "Foreign Obstacle": "square",
    }

    for cat in selected_categories:
        sub_df = df_filtered[df_filtered["category"] == cat]
        if not sub_df.empty:
            fig_map.add_trace(go.Scatter(
                x=sub_df["target_x"],
                y=sub_df["target_y"],
                mode="markers",
                marker=dict(
                    size=9,
                    color=color_map.get(cat, "#ffffff"),
                    symbol=symbol_map.get(cat, "circle"),
                    line=dict(color="#000000", width=1),
                    opacity=0.85
                ),
                name=f"{cat} ({len(sub_df)})",
                customdata=np.stack((
                    sub_df["event_id"],
                    sub_df["time"],
                    sub_df["distance_meters"],
                    sub_df["confidence"],
                    sub_df["target"],
                    sub_df["status"]
                ), axis=-1),
                hovertemplate=(
                    "<b>%{customdata[4]}</b><br>" +
                    "Event ID: #%{customdata[0]}<br>" +
                    "Time: %{customdata[1]}<br>" +
                    "Coordinates: (%{x:.2f}, %{y:.2f})<br>" +
                    "Range: %{customdata[2]:.2f} m<br>" +
                    "Confidence: %{customdata[3]:.0%}<br>" +
                    "Status: %{customdata[5]}" +
                    "<extra></extra>"
                )
            ))

    # Yard Annotations
    fig_map.add_annotation(x=0.0, y=1.8, text="North Container", showarrow=False, font=dict(color="#93c5fd", size=10))
    fig_map.add_annotation(x=0.0, y=-1.2, text="South Container", showarrow=False, font=dict(color="#fca5a5", size=10))
    fig_map.add_annotation(x=-3.6, y=-3.9, text="Guard Booth", showarrow=False, font=dict(color="#cbd5e1", size=10))
    fig_map.add_annotation(x=0.0, y=-5.1, text="Spawn Origin (0, -4.5)", showarrow=False, font=dict(color="#00ff88", size=10))

    fig_map.update_layout(
        template="plotly_dark",
        plot_bgcolor="#0d141f",
        paper_bgcolor="#0d141f",
        xaxis=dict(title="X Coordinate (Meters)", range=[-6.0, 6.0], zeroline=False, gridcolor="#1e293b"),
        yaxis=dict(title="Y Coordinate (Meters)", range=[-6.8, 6.8], zeroline=False, gridcolor="#1e293b", scaleanchor="x", scaleratio=1),
        height=620,
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        margin=dict(l=20, r=20, t=30, b=20)
    )

    st.plotly_chart(fig_map, use_container_width=True)


# -----------------------------------------------------------------------------
# TAB 2: SECURITY INCIDENT ANALYTICS
# -----------------------------------------------------------------------------
with tab_analytics:
    st.markdown("### 📊 Statistical Threat & Protocol Evaluation")
    
    col_a1, col_a2 = st.columns(2)
    
    with col_a1:
        st.markdown("##### Target Classification Breakdown")
        if not df_filtered.empty:
            cat_counts = df_filtered["category"].value_counts().reset_index()
            cat_counts.columns = ["Category", "Count"]
            fig_pie = px.pie(
                cat_counts,
                values="Count",
                names="Category",
                color="Category",
                color_discrete_map=color_map,
                hole=0.45
            )
            fig_pie.update_layout(
                template="plotly_dark",
                paper_bgcolor="#0d141f",
                plot_bgcolor="#0d141f",
                height=340,
                margin=dict(l=10, r=10, t=20, b=20),
            )
            st.plotly_chart(fig_pie, use_container_width=True)
        else:
            st.info("No data available for current filter.")

    with col_a2:
        st.markdown("##### Breach Range Distribution (Hazard Proximity)")
        if not df_filtered.empty:
            fig_hist = px.histogram(
                df_filtered,
                x="distance_meters",
                color="category",
                color_discrete_map=color_map,
                nbins=25,
                barmode="overlay",
                labels={"distance_meters": "Distance to Rover (m)"}
            )
            fig_hist.add_vline(x=2.5, line_width=2, line_dash="dash", line_color="#ef4444", annotation_text="Critical Hazard (<2.5m)")
            fig_hist.update_layout(
                template="plotly_dark",
                paper_bgcolor="#0d141f",
                plot_bgcolor="#0d141f",
                height=340,
                margin=dict(l=10, r=10, t=20, b=20),
                legend=dict(orientation="h", y=1.1, x=0)
            )
            st.plotly_chart(fig_hist, use_container_width=True)

    col_b1, col_b2 = st.columns(2)
    
    with col_b1:
        st.markdown("##### AI Vision Confidence Scores by Category")
        if not df_filtered.empty:
            fig_box = px.box(
                df_filtered,
                x="category",
                y="confidence",
                color="category",
                color_discrete_map=color_map,
                labels={"confidence": "YOLO Confidence", "category": "Target"}
            )
            fig_box.update_layout(
                template="plotly_dark",
                paper_bgcolor="#0d141f",
                plot_bgcolor="#0d141f",
                height=320,
                margin=dict(l=10, r=10, t=20, b=20),
                showlegend=False
            )
            st.plotly_chart(fig_box, use_container_width=True)

    with col_b2:
        st.markdown("##### Temporal Incident Density")
        if not df_filtered.empty and "hour" in df_filtered.columns and df_filtered["hour"].notna().any():
            hourly = df_filtered.groupby(["hour", "category"]).size().reset_index(name="counts")
            fig_time = px.bar(
                hourly,
                x="hour",
                y="counts",
                color="category",
                color_discrete_map=color_map,
                labels={"hour": "Hour of Day (24h)", "counts": "Detections"}
            )
            fig_time.update_layout(
                template="plotly_dark",
                paper_bgcolor="#0d141f",
                plot_bgcolor="#0d141f",
                height=320,
                margin=dict(l=10, r=10, t=20, b=20),
                legend=dict(orientation="h", y=1.1, x=0)
            )
            st.plotly_chart(fig_time, use_container_width=True)
        else:
            st.info("Time-series timestamps not populated in current subset.")


# -----------------------------------------------------------------------------
# TAB 3: DETAILED INTRUSION AUDIT LOG & EXPORT
# -----------------------------------------------------------------------------
with tab_table:
    st.markdown("### 📋 Security Audit Event Register")
    st.caption("Chronological record of geofence breaches, secondary rover evasions, and perimeter actions.")

    # Search Bar
    search_query = st.text_input("🔍 Search audit register (by target, date, time, or status):", "")
    table_df = df_filtered.copy()
    if search_query:
        q = search_query.lower()
        table_df = table_df[
            table_df["target"].astype(str).str.lower().str.contains(q)
            | table_df["date"].astype(str).str.lower().str.contains(q)
            | table_df["time"].astype(str).str.lower().str.contains(q)
            | table_df["status"].astype(str).str.lower().str.contains(q)
        ]

    # Present formatted table
    display_cols = [
        "event_id", "date", "time", "target", "category",
        "distance_meters", "confidence", "rover_x", "rover_y",
        "target_x", "target_y", "status"
    ]
    renamed_cols = {
        "event_id": "# ID",
        "date": "Date",
        "time": "Time",
        "target": "Spotted Target",
        "category": "Classification",
        "distance_meters": "Range (m)",
        "confidence": "Confidence",
        "rover_x": "Rover X",
        "rover_y": "Rover Y",
        "target_x": "Target X",
        "target_y": "Target Y",
        "status": "Security Protocol Action",
    }
    
    st.dataframe(
        table_df[display_cols].rename(columns=renamed_cols).sort_values("# ID", ascending=False),
        use_container_width=True,
        height=450,
    )

    # Data Export Buttons
    col_exp1, col_exp2, col_exp_spacer = st.columns([0.2, 0.2, 0.6])
    with col_exp1:
        csv_data = table_df.to_csv(index=False).encode("utf-8")
        st.download_button(
            label="📥 Export Filtered CSV",
            data=csv_data,
            file_name=f"perimeter_audit_log_{datetime.now().strftime('%Y%m%d')}.csv",
            mime="text/csv",
        )
    with col_exp2:
        json_data = table_df.to_json(orient="records", indent=2)
        st.download_button(
            label="📥 Export Filtered JSON",
            data=json_data,
            file_name=f"perimeter_audit_log_{datetime.now().strftime('%Y%m%d')}.json",
            mime="application/json",
        )


# -----------------------------------------------------------------------------
# TAB 4: SURVEILLANCE & VISUAL EVIDENCE GALLERY
# -----------------------------------------------------------------------------
with tab_evidence:
    st.markdown("### 📸 Visual Evidence & Autonomous Telemetry Gallery")
    st.caption("Snapshot evidence captured during perimeter surveillance patrol.")

    col_img1, col_img2 = st.columns(2)

    with col_img1:
        # Display current rover view or simulation snapshots
        img_path = os.path.join(os.path.dirname(__file__), "current_rover_view.png")
        if os.path.exists(img_path):
            st.image(img_path, caption="Live Rover Surveillance Camera View (YOLO Geofenced)", use_container_width=True)
        else:
            st.info("Live snapshot 'current_rover_view.png' pending generation.")

        fig_dual = os.path.join(os.path.dirname(__file__), "report_assets/custom_figures/fig7_1_dual_tactical_monitor.png")
        if os.path.exists(fig_dual):
            st.image(fig_dual, caption="Dual Tactical Monitor (Camera Stream + Top-View Radar)", use_container_width=True)

    with col_img2:
        fig_route = os.path.join(os.path.dirname(__file__), "report_assets/custom_figures/fig8_1_nav2_patrol_route.png")
        if os.path.exists(fig_route):
            st.image(fig_route, caption="Nav2 Perimeter Checkpoint Route & Costmap Inflation", use_container_width=True)

        fig_audit = os.path.join(os.path.dirname(__file__), "report_assets/custom_figures/fig9_1_audit_trail_verification.png")
        if os.path.exists(fig_audit):
            st.image(fig_audit, caption="Audit Trail Verification & Live Incident Log", use_container_width=True)


# -----------------------------------------------------------------------------
# TAB 5: SYSTEM SPECIFICATIONS & FAILSAFES
# -----------------------------------------------------------------------------
with tab_arch:
    st.markdown("### ⚙️ Autonomous Rover Technical Specifications")
    
    col_s1, col_s2 = st.columns(2)
    with col_s1:
        st.markdown(
            """
            #### 🤖 Platform & Kinematics
            - **Chassis**: TurtleBot3 Waffle Differential Drive
            - **Max Linear Speed**: 0.26 m/s (Configurable via Web Teleop)
            - **Max Angular Speed**: 1.82 rad/s
            - **LiDAR**: 360° 2D Laser Scanner (LDS-01)
            - **Camera**: RGB 640×480 Front Optical Bridge (`/camera/image_raw`)
            - **Sim Environment**: Gazebo Harmonic (Sim 8) with Ogre2 physics
            
            #### 🧭 Nav2 Navigation Stack
            - **Controller**: Model Predictive Path Integral (MPPI)
            - **Planner**: NavFn / Smac Planner
            - **Costmaps**: 2D Static Map Layer + Obstacle Laser Layer + Inflation
            - **Recovery**: Automatic Costmap Flush & Dynamic Fallback
            """
        )
    with col_s2:
        st.markdown(
            """
            #### 🛡️ Autonomous Failsafe Protocols
            - **Human Intruder Protocol**:
              - *Action*: Immediate velocity cutoff (`Twist() = 0`)
              - *Behavior*: Halt and Hold perimeter boundary
              - *Alerts*: Loud terminal alert + Linux desktop notification + JSON log
              - *Resolution*: Operator Resume (`'R'`) with 35s bypass grace period
            - **Secondary Rover Protocol**:
              - *Action*: Autonomous evasion (`_execute_yield_and_pass_maneuver`)
              - *Behavior*: Steer to corridor shoulder, yield right-of-way, resume route
            - **Mission Control Web Teleop**:
              - Dual-Mode Switcher: Autonomous Patrol $\\leftrightarrow$ Manual Joystick
              - Emergency Stop: Instant brake & Total System Reset to Origin $(0.0, -4.50)$
            """
        )


# -----------------------------------------------------------------------------
# FOOTER
# -----------------------------------------------------------------------------
st.markdown("---")
st.markdown(
    """
    <div style="text-align: center; color: #64748b; font-size: 0.8rem; font-family: monospace;">
        Autonomous Perimeter Security Rover System &bull; ROS 2 Jazzy &bull; Nav2 MPPI &bull; Ultralytics YOLOv8 &bull; Gazebo Harmonic<br>
        Deployed via Streamlit Community Cloud
    </div>
    """,
    unsafe_allow_html=True,
)
