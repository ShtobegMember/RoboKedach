"""
hud_painter.py - HUD Rendering Logic
Separates the tactical Qt drawing logic from the window management.
Contains the exact pixel-perfect layout of the original RoboKedach HUD.
"""

from PyQt6.QtGui import QPainter, QColor, QPen, QFont
from PyQt6.QtCore import Qt

class HUDPainter:
    @staticmethod
    def _draw_leg_box(p, x, y, sz, active, green, white, panel_bg):
        """Draw a single leg indicator box with a down-arrow."""
        color = green if active else white
        p.setPen(QPen(color, 1.5))
        p.setBrush(panel_bg)
        p.drawRoundedRect(x, y, sz, sz, 3, 3)

        # Draw down arrow inside the box
        cx = x + sz // 2
        top_y = y + 5
        bot_y = y + sz - 5

        p.setPen(QPen(color, 2))
        p.drawLine(cx, top_y, cx, bot_y)            # shaft
        p.drawLine(cx, bot_y, cx - 4, bot_y - 5)    # left head
        p.drawLine(cx, bot_y, cx + 4, bot_y - 5)    # right head

    @staticmethod
    def paint_hud(widget, state):
        """Main entry point. Pulls state from HUDWindow and paints the original layout."""
        p = QPainter(widget)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = widget.width(), widget.height()

        # Colors
        panel_bg = QColor(0, 0, 0, 140)
        green = QColor(0, 255, 0, 220)
        white = QColor(255, 255, 255, 240)
        yellow = QColor(255, 255, 0, 220)
        orange = QColor(255, 165, 0, 220)
        red = QColor(255, 0, 0, 220)
        border = QPen(green, 1)

        title_font = QFont("Consolas", 11, QFont.Weight.Bold)
        status_font = QFont("Consolas", 9)

        # --- Crosshair (center of screen) ---
        cx, cy = w // 2, h // 2
        solid_len = 22    # solid segment length from center
        gap       = 7     # gap around center point

        solid_pen = QPen(QColor(0, 255, 0, 240), 3.5)
        solid_pen.setCapStyle(Qt.PenCapStyle.RoundCap)

        dot_pen = QPen(QColor(0, 255, 0, 170), 2.5)
        dot_pen.setStyle(Qt.PenStyle.CustomDashLine)
        dot_pen.setDashPattern([8, 6])   # 8px dash, 6px gap
        dot_pen.setCapStyle(Qt.PenCapStyle.RoundCap)

        # Solid arms
        p.setPen(solid_pen)
        p.drawLine(cx - solid_len - gap, cy, cx - gap, cy)   # left solid
        p.drawLine(cx + gap, cy, cx + solid_len + gap, cy)   # right solid
        p.drawLine(cx, cy - solid_len - gap, cx, cy - gap)   # top solid
        p.drawLine(cx, cy + gap, cx, cy + solid_len + gap)   # bottom solid

        # Edge tick marks
        tick = 18          # tick length in px
        margin = 20        # distance from screen edge
        p.drawLine(margin, cy, margin + tick, cy)              # left edge
        p.drawLine(w - margin - tick, cy, w - margin, cy)      # right edge
        p.drawLine(cx, margin, cx, margin + tick)              # top edge
        p.drawLine(cx, h - 4*margin - tick, cx, h - 4*margin)  # bottom edge

        # --- Power Monitor Panel (top-left) ---
        pm_y = 10
        p.setBrush(panel_bg)
        p.setPen(border)
        p.drawRoundedRect(10, pm_y, 190, 80, 8, 8)

        p.setFont(title_font)
        p.setPen(green)
        p.drawText(22, pm_y + 24, "POWER MONITOR")

        p.setFont(QFont("Consolas", 12, QFont.Weight.Bold))
        vm = state.vm_data
        p.setPen(white)
        p.drawText(22, pm_y + 50, f"{vm['voltage']:6.2f} V")
        p.drawText(22, pm_y + 70, f"{vm['current']:6.2f} A")

        # Voltage battery indicator
        v = vm['voltage']
        if v > 0:
            bx = 150
            by = pm_y + 22
            bw = 26
            bh = 42
            cap_w = 10
            cap_h = 4

            # Outline and cap
            p.setPen(QPen(white, 2))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRoundedRect(bx, by, bw, bh, 2, 2)
            p.setBrush(white)
            p.setPen(Qt.PenStyle.NoPen)
            p.drawRect(bx + (bw - cap_w) // 2, by - cap_h, cap_w, cap_h)

            if v > 11.7:
                level = 4
                bat_color = green
            elif v > 11.2:
                level = 3
                bat_color = yellow
            elif v > 10.8:
                level = 2
                bat_color = orange
            elif v > 10.5:
                level = 1
                bat_color = red
            else:
                level = 0
                bat_color = red

            if level > 0:
                pad = 2
                seg_gap = 2
                seg_h = (bh - 2 * pad - 3 * seg_gap) // 4
                seg_w = bw - 2 * pad
                p.setBrush(bat_color)
                for i in range(level):
                    seg_x = bx + pad
                    seg_y = by + bh - pad - (i + 1) * seg_h - i * seg_gap
                    p.drawRect(seg_x, seg_y, int(seg_w), int(seg_h))

        p.setBrush(panel_bg)

        # Layout constants for status bar
        sb_h = 50
        sb_y = h - sb_h - 10

        # --- Differential Speed Panel (bottom-left, above status bar) ---
        sp_w = 140
        sp_h = 120
        sp_x = 10
        sp_y = sb_y - 10 - sp_h  # 10px gap above status bar
        p.setBrush(panel_bg)
        p.setPen(border)
        p.drawRoundedRect(sp_x, sp_y, sp_w, sp_h, 8, 8)

        p.setFont(title_font)
        p.setPen(green)
        p.drawText(sp_x + 12, sp_y + 22, "SPEED")

        # Bar track geometry
        bar_w = 14
        bar_top = sp_y + 35
        bar_bottom = sp_y + sp_h - 26
        bar_h = bar_bottom - bar_top

        # Three evenly-spaced columns (left=L, middle=avg, right=R)
        col_centers = [sp_x + 28, sp_x + 66, sp_x + 104]

        # Range labels (top=127, bottom=10)
        range_font = QFont("Consolas", 8)
        p.setFont(range_font)
        p.setPen(white)
        p.drawText(sp_x + sp_w - 25, bar_top + 6, str(state.SPEED_MAX))
        p.drawText(sp_x + sp_w - 20, bar_bottom + 4, str(state.SPEED_MIN))

        # Slot values and colors
        lo, hi = state.SPEED_MIN, state.SPEED_MAX
        left_val = state.motor_left_speed
        right_val = state.motor_right_speed
        avg_val = (left_val + right_val) // 2

        green_active = green                       # sides — bright green
        green_muted = QColor(0, 180, 0, 110)       # middle average — grayed-out green

        slots = [
            (col_centers[0], left_val, green_active),
            (col_centers[1], avg_val, green_muted),
            (col_centers[2], right_val, green_active),
        ]

        track_bg = QColor(255, 255, 255, 40)
        for cx, val, fill_color in slots:
            bx = cx - bar_w // 2
            # Track
            p.setPen(QPen(green, 1))
            p.setBrush(track_bg)
            p.drawRect(bx, bar_top, bar_w, bar_h)

            # Fill proportional to (val - lo) / (hi - lo), clamped
            if hi > lo:
                frac = max(0.0, min(1.0, (val - lo) / float(hi - lo)))
            else:
                frac = 0.0
            fill_h = int(bar_h * frac)
            if fill_h > 0:
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(fill_color)
                p.drawRect(bx + 1, bar_bottom - fill_h + 1, bar_w - 2, fill_h - 1)

            # Numeric value below bar
            p.setPen(white)
            p.setFont(range_font)
            p.drawText(cx - 10, bar_bottom + 16, f"{val:>3d}")

        p.setBrush(panel_bg)

        # --- Robot Legs Panel (bottom-right, above status bar) ---
        lp_w = 140
        lp_h = 180
        lp_x = w - lp_w - 10
        lp_y = sb_y - 10 - lp_h

        p.setBrush(panel_bg)
        p.setPen(border)
        p.drawRoundedRect(lp_x, lp_y, lp_w, lp_h, 8, 8)

        p.setFont(title_font)
        p.setPen(green)
        p.drawText(lp_x + 12, lp_y + 22, "LEGS STATUS")

        # Robot body illustration (centered in panel)
        body_w = 36
        body_h = 120
        body_x = lp_x + (lp_w - body_w) // 2
        body_y = lp_y + 42

        p.setPen(QPen(white, 2))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(body_x, body_y, body_w, body_h, 6, 6)

        # Camera nub on top
        nub_w = 16
        nub_h = 6
        p.setBrush(white)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRect(body_x + (body_w - nub_w) // 2, body_y - nub_h, nub_w, nub_h)

        # Forward arrow inside body
        arr_cx = body_x + body_w // 2
        arr_cy = body_y + 22
        p.setPen(QPen(white, 2))
        p.drawLine(arr_cx, arr_cy + 14, arr_cx, arr_cy - 8)           # shaft
        p.drawLine(arr_cx, arr_cy - 8, arr_cx - 6, arr_cy - 1)       # left head
        p.drawLine(arr_cx, arr_cy - 8, arr_cx + 6, arr_cy - 1)       # right head

        # Camera lens (small square + circle) inside body
        lens_y = arr_cy + 22
        lens_sz = 16
        p.setPen(QPen(white, 1.5))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(arr_cx - lens_sz // 2, lens_y, lens_sz, lens_sz, 2, 2)
        p.drawEllipse(arr_cx - 5, lens_y + 3, 10, 10)

        # Cable dot at bottom
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(white)
        p.drawEllipse(arr_cx - 3, body_y + body_h - 12, 6, 6)

        # Cable line hanging below robot body
        cable_top_y = body_y + body_h
        cable_bot_y = lp_y + lp_h - 6
        cable_pen = QPen(white, 1.5)
        cable_pen.setStyle(Qt.PenStyle.DashLine)
        p.setPen(cable_pen)
        p.drawLine(arr_cx, cable_top_y, arr_cx, cable_bot_y)

        # --- Leg arrow boxes (3 on each side) ---
        box_sz = 22
        leg_offsets = state.LEG_OFFSETS   # [L1, L2, L3, R1, R2, R3]
        phase_l = state.leg_phase_left
        phase_r = state.leg_phase_right
        leg_cycle = [False, True, False, False]  # white, GREEN, white, white

        # Vertical positions for the 3 rows of legs (front, mid, rear)
        leg_ys = [
            body_y + 8,
            body_y + (body_h - box_sz) // 2,
            body_y + body_h - box_sz - 8,
        ]

        left_x  = body_x - box_sz - 10
        right_x = body_x + body_w + 10

        for i in range(3):
            # Left leg (indices 0-2) — uses right phase
            l_active = leg_cycle[(phase_r - leg_offsets[i]) % 4]
            HUDPainter._draw_leg_box(p, left_x, leg_ys[i], box_sz, l_active, green, QColor(255, 255, 255, 120), panel_bg)
            # Right leg (indices 3-5) — uses left phase
            r_active = leg_cycle[(phase_l - leg_offsets[i + 3]) % 4]
            HUDPainter._draw_leg_box(p, right_x, leg_ys[i], box_sz, r_active, green, QColor(255, 255, 255, 120), panel_bg)

        # --- Status Bar (bottom) ---
        p.setBrush(QColor(0, 0, 0, 160))
        p.setPen(border)
        p.drawRoundedRect(10, sb_y, w - 20, sb_h, 5, 5)

        p.setFont(status_font)
        p.setPen(green)

        # Derive simplified status strings
        cam_raw = state.camera_status.lower()
        cam_st = "Connected" if "connected" in cam_raw else "Disconnected"

        vm_raw = getattr(state, 'vm_status', '')
        vm_st = "Connected" if "connected" in vm_raw.lower() else "Disconnected"

        motor_raw = state.motor_status.lower()
        motor_st = "Disconnected" if "disconnected" in motor_raw else "Connected"

        bold_font   = QFont("Consolas", 9, QFont.Weight.Bold)
        normal_font = QFont("Consolas", 9)
        label_gap   = 55   # pixels to skip past the bold label

        # Row 1 — BASE:
        p.setFont(bold_font)
        p.drawText(20, sb_y + 20, "BASE:")
        p.setFont(normal_font)
        p.drawText(20 + label_gap, sb_y + 20, f"COMM: OK  |  {state.heading_status}  |  {state.slam_status}")

        # Row 2 — ROBOT:
        p.setFont(bold_font)
        p.drawText(20, sb_y + 40, "ROBOT:")
        p.setFont(normal_font)
        p.drawText(20 + label_gap, sb_y + 40, f"VM: {vm_st}  |  CAMERA: {cam_st}  |  MOTORS: {motor_st}")

        p.end()