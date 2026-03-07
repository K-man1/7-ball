"""
visualize_mcl.py — MCL particle visualizer for 30000S-RoboSapiens
------------------------------------------------------------------
Usage:
  1. Pull mcl_log.csv off the SD card and put it in the same folder as this script
  2. pip install matplotlib numpy
  3. python visualize_mcl.py

Controls:
  - Use the slider at the bottom to scrub through time steps
  - Particles are colored by weight (dark = low weight, bright yellow = high weight)
  - Red X marks the MCL estimated position at that tick
  - Field obstacles (loaders, goals, center bar) are shown in gray
"""

import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Slider
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable

# ─── Field constants (inches) ─────────────────────────────────────────────────
FIELD_SIZE = 140.42
HALF = FIELD_SIZE / 2

LOADER_X = 47.0
LOADER_RADIUS = 3.0
LOADER_PADDING = 0.5
LOADER_W = LOADER_RADIUS * 2 + LOADER_PADDING * 2
LOADER_H = LOADER_RADIUS * 2 + LOADER_PADDING

GOAL_X, GOAL_Y = 48.0, 23.0
GOAL_PADDING = 1.0
GOAL_W = 6.0 + GOAL_PADDING * 2
GOAL_H = 3.0 + GOAL_PADDING * 2

CENTER_W = CENTER_H = 21.0 + 2.0

LOADERS = [
    (-LOADER_X, -(FIELD_SIZE / 2) + LOADER_RADIUS + LOADER_PADDING / 2),
    (-LOADER_X,  (FIELD_SIZE / 2) - LOADER_RADIUS - LOADER_PADDING / 2),
    ( LOADER_X,  (FIELD_SIZE / 2) - LOADER_RADIUS - LOADER_PADDING / 2),
    ( LOADER_X, -(FIELD_SIZE / 2) + LOADER_RADIUS + LOADER_PADDING / 2),
]

GOALS = [
    (-GOAL_X, -GOAL_Y), (-GOAL_X, GOAL_Y),
    ( GOAL_X,  GOAL_Y), ( GOAL_X, -GOAL_Y),
]

# ─── Load CSV ─────────────────────────────────────────────────────────────────
ticks = []  # list of (particles_xy, weights, estimate_xy)

current_particles = []
current_weights = []

with open("mcl_log.csv", "r") as f:
    reader = csv.reader(f)
    for row in reader:
        if not row:
            continue
        if row[0] == "P":
            current_particles.append((float(row[1]), float(row[2])))
            current_weights.append(float(row[3]))
        elif row[0] == "E":
            est = (float(row[1]), float(row[2]))
            ticks.append((
                np.array(current_particles),
                np.array(current_weights),
                est
            ))
            current_particles = []
            current_weights = []

print(f"Loaded {len(ticks)} ticks")

if not ticks:
    print("No data found in mcl_log.csv")
    exit()

# ─── Plot ─────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(8, 8))
plt.subplots_adjust(bottom=0.15)

def draw_field(ax):
    # Field border
    ax.set_xlim(-HALF - 5, HALF + 5)
    ax.set_ylim(-HALF - 5, HALF + 5)
    ax.set_aspect("equal")
    ax.set_facecolor("#2a2a2a")
    field_rect = patches.Rectangle((-HALF, -HALF), FIELD_SIZE, FIELD_SIZE,
                                    linewidth=2, edgecolor="white", facecolor="#3a3a3a")
    ax.add_patch(field_rect)

    # Loaders
    for lx, ly in LOADERS:
        r = patches.Rectangle((lx - LOADER_W/2, ly - LOADER_H/2), LOADER_W, LOADER_H,
                               facecolor="#888", edgecolor="white", linewidth=0.5)
        ax.add_patch(r)

    # Goals
    for gx, gy in GOALS:
        r = patches.Rectangle((gx - GOAL_W/2, gy - GOAL_H/2), GOAL_W, GOAL_H,
                               facecolor="#666", edgecolor="white", linewidth=0.5)
        ax.add_patch(r)

    # Center bar
    r = patches.Rectangle((-CENTER_W/2, -CENTER_H/2), CENTER_W, CENTER_H,
                           facecolor="#555", edgecolor="white", linewidth=0.5)
    ax.add_patch(r)

    ax.set_xlabel("X (inches)")
    ax.set_ylabel("Y (inches)")
    ax.grid(True, color="#444", linewidth=0.3)

draw_field(ax)

# Initial scatter
pts, wts, est = ticks[0]
norm = Normalize(vmin=0, vmax=np.max(wts) if np.max(wts) > 0 else 1)
colors = plt.cm.plasma(norm(wts))

scat = ax.scatter(pts[:, 0], pts[:, 1], c=colors, s=2, alpha=0.7, zorder=3)
est_dot, = ax.plot(est[0], est[1], "rx", markersize=12, markeredgewidth=2, zorder=5)
title = ax.set_title(f"Tick 0 / {len(ticks)-1}  |  Est: ({est[0]:.1f}, {est[1]:.1f})")

# Slider
ax_slider = plt.axes([0.15, 0.05, 0.7, 0.03])
slider = Slider(ax_slider, "Tick", 0, len(ticks) - 1, valinit=0, valstep=1)

def update(val):
    tick = int(slider.val)
    pts, wts, est = ticks[tick]
    norm = Normalize(vmin=0, vmax=np.max(wts) if np.max(wts) > 0 else 1)
    colors = plt.cm.plasma(norm(wts))
    scat.set_offsets(pts)
    scat.set_color(colors)
    est_dot.set_data([est[0]], [est[1]])
    title.set_text(f"Tick {tick} / {len(ticks)-1}  |  Est: ({est[0]:.1f}, {est[1]:.1f})")
    fig.canvas.draw_idle()

slider.on_changed(update)
plt.suptitle("MCL Particle Visualizer — 30000S-RoboSapiens", color="white",
             fontsize=11, fontweight="bold")
fig.patch.set_facecolor("#1a1a1a")
ax.tick_params(colors="white")
ax.xaxis.label.set_color("white")
ax.yaxis.label.set_color("white")
title.set_color("white")
plt.show()