"""
Utility to find board crop coordinates or calibrate pixels_per_mm.

Usage:
    ./venv/bin/python3 find_coords.py            -> board crop mode (full image, 4 corner clicks)
    ./venv/bin/python3 find_coords.py --ppmm     -> ppmm mode (cropped to board, 2 corner clicks)
    ./venv/bin/python3 find_coords.py [image.png] [--ppmm]

Board crop mode: click all 4 corners of the plywood (any order: TL, TR, BR, BL).
PPMM mode: click top-left then bottom-right of the object.
"""

import tkinter as tk
from PIL import Image, ImageTk
import os
import sys
import json

UPLOADS_DIR = os.path.join(os.path.dirname(__file__), "uploads")
CONFIG_FILE  = os.path.join(UPLOADS_DIR, "config.json")
SCALE = 3

# Parse args
ppmm_mode = "--ppmm" in sys.argv
args = [a for a in sys.argv[1:] if not a.startswith("--")]

# Pick image
if args:
    img_path = args[0]
else:
    pngs = sorted([f for f in os.listdir(UPLOADS_DIR) if f.endswith(".png")])
    if not pngs:
        print("No PNG files found in uploads/")
        sys.exit(1)
    img_path = os.path.join(UPLOADS_DIR, pngs[-1])

print(f"Opening: {img_path}")
img = Image.open(img_path).convert("L")
print(f"Full image size: {img.width} x {img.height} px")

# Load board crop from config if in ppmm mode
crop_offset = (0, 0)
if ppmm_mode and os.path.exists(CONFIG_FILE):
    with open(CONFIG_FILE) as f:
        cfg = json.load(f)
    # Support both old rect format and new 4-point format
    if "board_p0x" in cfg:
        xs = [cfg[f"board_p{i}x"] for i in range(4)]
        ys = [cfg[f"board_p{i}y"] for i in range(4)]
        x0, y0, x1, y1 = min(xs), min(ys), max(xs), max(ys)
    else:
        x0 = cfg.get("board_x0", 0)
        y0 = cfg.get("board_y0", 0)
        x1 = cfg.get("board_x1", img.width)
        y1 = cfg.get("board_y1", img.height)
    img = img.crop((x0, y0, x1, y1))
    crop_offset = (x0, y0)
    print(f"Cropped to board bbox: ({x0},{y0}) → ({x1},{y1})  =  {img.width}x{img.height} px")

img_big = img.resize((img.width * SCALE, img.height * SCALE), Image.NEAREST)
clicks = []

root = tk.Tk()
mode_label = "PPMM calibration — cropped board" if ppmm_mode else "Board crop — click all 4 corners of the plywood"
root.title(mode_label)

CORNER_NAMES = ["TOP-LEFT", "TOP-RIGHT", "BOTTOM-RIGHT", "BOTTOM-LEFT"]

if ppmm_mode:
    prompt = "Click TOP-LEFT corner of the OBJECT"
else:
    prompt = f"Click corner 1/4 — {CORNER_NAMES[0]} of the PLYWOOD"

status = tk.Label(root, text=prompt, fg="white", bg="black",
                  font=("monospace", 12), pady=6)
status.pack(fill="x")

tk_img = ImageTk.PhotoImage(img_big)
canvas = tk.Canvas(root, width=img_big.width, height=img_big.height, cursor="crosshair")
canvas.pack()
canvas.create_image(0, 0, anchor="nw", image=tk_img)

coord_label = tk.Label(root, text="x=0, y=0", fg="#aaa", bg="black",
                       font=("monospace", 11), pady=4)
coord_label.pack(fill="x")


def on_move(e):
    x, y = e.x // SCALE, e.y // SCALE
    coord_label.config(text=f"x={x},  y={y}")


def draw_polygon(pts):
    """Draw lines connecting the 4 clicked points."""
    if len(pts) < 2:
        return
    for i in range(len(pts)):
        if i + 1 < len(pts):
            x0, y0 = pts[i][0] * SCALE, pts[i][1] * SCALE
            x1, y1 = pts[i+1][0] * SCALE, pts[i+1][1] * SCALE
            canvas.create_line(x0, y0, x1, y1, fill="lime", width=2)
    if len(pts) == 4:
        x0, y0 = pts[3][0] * SCALE, pts[3][1] * SCALE
        x1, y1 = pts[0][0] * SCALE, pts[0][1] * SCALE
        canvas.create_line(x0, y0, x1, y1, fill="lime", width=2)


def on_click(e):
    x, y = e.x // SCALE, e.y // SCALE
    clicks.append((x, y))
    n = len(clicks)
    canvas.create_oval(e.x-5, e.y-5, e.x+5, e.y+5, fill="red", outline="red")
    canvas.create_text(e.x+10, e.y, text=f"P{n-1}({x},{y})", fill="red", anchor="w")

    if ppmm_mode:
        if n == 1:
            print(f"Object top-left:     x={x}, y={y}")
            status.config(text="Click BOTTOM-RIGHT corner of the OBJECT")
        elif n == 2:
            x0, y0 = clicks[0]
            x1, y1 = clicks[1]
            w_px = abs(x1 - x0)
            h_px = abs(y1 - y0)
            print(f"Object bottom-right: x={x1}, y={y1}")
            print()
            print("=" * 42)
            print(f"  Object size in pixels: {w_px} x {h_px} px")
            print()
            print("  Enter actual object dimensions:")
            try:
                actual_w = float(input("  Actual width  (mm): "))
                actual_h = float(input("  Actual height (mm): "))
                ppmm_w = w_px / actual_w
                ppmm_h = h_px / actual_h
                ppmm_avg = (ppmm_w + ppmm_h) / 2
                print()
                print(f"  ppmm from width:  {ppmm_w:.4f}")
                print(f"  ppmm from height: {ppmm_h:.4f}")
                print(f"  Average ppmm:     {ppmm_avg:.4f}  ← use this")
                print("=" * 42)
                status.config(text=f"ppmm = {ppmm_avg:.4f} — enter in frontend Board Calibration")
            except ValueError:
                print("Invalid input.")
    else:
        print(f"P{n-1} ({CORNER_NAMES[n-1]}): x={x}, y={y}")
        draw_polygon(clicks)

        if n < 4:
            status.config(text=f"Click corner {n+1}/4 — {CORNER_NAMES[n]} of the PLYWOOD")
        else:
            p = clicks
            print()
            print("=" * 44)
            print("  Board polygon coordinates:")
            print(f"  P0 (TL): x={p[0][0]}, y={p[0][1]}")
            print(f"  P1 (TR): x={p[1][0]}, y={p[1][1]}")
            print(f"  P2 (BR): x={p[2][0]}, y={p[2][1]}")
            print(f"  P3 (BL): x={p[3][0]}, y={p[3][1]}")
            print()
            print("  Enter these in the frontend Board Calibration section.")
            print("=" * 44)
            status.config(text=f"Done! P0({p[0][0]},{p[0][1]}) P1({p[1][0]},{p[1][1]}) P2({p[2][0]},{p[2][1]}) P3({p[3][0]},{p[3][1]})")


canvas.bind("<Motion>", on_move)
canvas.bind("<Button-1>", on_click)
root.mainloop()
