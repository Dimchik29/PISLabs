import csv
import json
import math
import os
import random
import sys
from datetime import datetime

import pygame


WIDTH, HEIGHT = 1440, 860
FPS = 60

RIGHT_W = 365
TOP_H = 62
BOTTOM_H = 220

VIEW = pygame.Rect(20, TOP_H + 10, WIDTH - RIGHT_W - 55, HEIGHT - TOP_H - BOTTOM_H - 35)
SIDE = pygame.Rect(WIDTH - RIGHT_W - 18, 20, RIGHT_W, HEIGHT - 40)

MAP = pygame.Rect(20, HEIGHT - BOTTOM_H - 5, 400, BOTTOM_H - 15)
GRAPH = pygame.Rect(440, HEIGHT - BOTTOM_H - 5, 270, BOTTOM_H - 15)
LOGBOX = pygame.Rect(730, HEIGHT - BOTTOM_H - 5, 310, BOTTOM_H - 15)

WORLD_W, WORLD_H = 1400.0, 850.0

BG = (7, 13, 23)
PANEL = (16, 27, 42)
PANEL2 = (21, 39, 60)
BORDER = (56, 101, 135)
TEXT = (232, 243, 255)
MUTED = (151, 176, 196)
WHITE = (255, 255, 255)
ORANGE = (255, 132, 28)
ORANGE2 = (221, 85, 18)
GREEN = (62, 224, 132)
RED = (255, 76, 76)
YELLOW = (255, 210, 88)
BLUE = (64, 165, 245)
CYAN = (84, 235, 255)
WATER = (8, 55, 85)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def wrap_angle(deg):
    """Return angle in range [-180, 180]."""
    while deg > 180:
        deg -= 360
    while deg < -180:
        deg += 360
    return deg


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def ensure_outputs():
    os.makedirs("outputs", exist_ok=True)


class Fonts:
    def __init__(self):
        pygame.font.init()
        self.small = pygame.font.SysFont("segoeui,arial,tahoma", 15)
        self.normal = pygame.font.SysFont("segoeui,arial,tahoma", 18)
        self.medium = pygame.font.SysFont("segoeui,arial,tahoma", 22, bold=True)
        self.big = pygame.font.SysFont("segoeui,arial,tahoma", 34, bold=True)

    def draw(self, surface, text, pos, color=TEXT, font=None, center=False):
        font = font or self.normal
        image = font.render(str(text), True, color)
        rect = image.get_rect()
        if center:
            rect.center = pos
        else:
            rect.topleft = pos
        surface.blit(image, rect)
        return rect


class Button:
    def __init__(self, rect, label, command):
        self.rect = pygame.Rect(rect)
        self.label = label
        self.command = command
        self.active = False
        self.enabled = True

    def draw(self, surface, fonts):
        mouse = pygame.mouse.get_pos()
        hover = self.rect.collidepoint(mouse)
        if not self.enabled:
            fill = (42, 47, 55)
            border = (70, 70, 75)
            color = MUTED
        elif self.active:
            fill = (18, 116, 91)
            border = GREEN
            color = WHITE
        elif hover:
            fill = (37, 67, 94)
            border = CYAN
            color = WHITE
        else:
            fill = PANEL2
            border = BORDER
            color = TEXT

        pygame.draw.rect(surface, fill, self.rect, border_radius=10)
        pygame.draw.rect(surface, border, self.rect, 1, border_radius=10)
        fonts.draw(surface, self.label, self.rect.center, color, fonts.small, center=True)

    def clicked(self, event):
        return (
            self.enabled
            and event.type == pygame.MOUSEBUTTONDOWN
            and event.button == 1
            and self.rect.collidepoint(event.pos)
        )


class EventLog:
    def __init__(self, max_items=9):
        self.max_items = max_items
        self.items = []

    def add(self, text):
        stamp = datetime.now().strftime("%H:%M:%S")
        self.items.insert(0, f"[{stamp}] {text}")
        self.items = self.items[: self.max_items]


class HydroBoat:
    """
    Simulation model.

    Important coordinate rule:
    - world x grows to the right
    - world y grows downward
    - heading 0° = east/right
    - heading 90° = south/down
    This removes the old autopilot sign error.
    """

    def __init__(self):
        self.x = 185.0
        self.y = 555.0
        self.home = (self.x, self.y)
        self.heading = 0.0
        self.speed = 0.0
        self.target_speed = 0.0
        self.max_speed = 7.0
        self.turn_rate_limit = 115.0

        self.mode = "MANUAL"
        self.recording = False
        self.sonar_on = True
        self.adcp_on = True
        self.rtk_on = True
        self.imu_ok = True

        self.depth = 0.0
        self.temperature = 18.0
        self.battery = 100.0
        self.link = 100.0
        self.gps_status = "RTK FIX"
        self.gps_satellites = 18
        self.roll = 0.0
        self.pitch = 0.0
        self.current = (0.0, 0.0)
        self.left_motor = 0.0
        self.right_motor = 0.0

        self.track = []
        self.samples = []
        self.scanned_cells = set()
        self.coverage_m2 = 0.0
        self.last_sample_time = 0.0

        self.route = []
        self.wp_index = 0
        self.accept_radius = 26.0
        self.final_accept_radius = 18.0
        self.hold_position = (self.x, self.y)

        self.debug_target = None
        self.debug_error = 0.0
        self.debug_distance = 0.0

    def set_manual(self):
        self.mode = "MANUAL"
        self.route = []
        self.wp_index = 0

    def set_autopilot(self, route):
        self.route = list(route)
        self.wp_index = 0
        self.mode = "AUTOPILOT"
        self.recording = True

    def set_return_home(self):
        self.route = [self.home]
        self.wp_index = 0
        self.mode = "RETURN"
        self.recording = False

    def set_hold(self):
        self.mode = "HOLD"
        self.route = []
        self.wp_index = 0
        self.hold_position = (self.x, self.y)
        self.target_speed = 0.0

    def stop(self):
        self.target_speed = 0.0
        self.speed *= 0.55

    def terrain_depth(self):
        nx = self.x / WORLD_W
        ny = self.y / WORLD_H
        basin = 8.0 + 9.0 * math.sin(nx * math.pi) * math.sin(ny * math.pi)
        ridges = 1.5 * math.sin(nx * 17.0 + ny * 4.0)
        local = 0.8 * math.sin((self.x + self.y) * 0.018)
        return clamp(basin + ridges + local, 1.0, 24.0)

    def water_current(self, t):
        return (
            0.10 * math.sin(self.y * 0.012 + t * 0.30),
            0.08 * math.cos(self.x * 0.010 + t * 0.25),
        )

    def update_manual(self, keys, dt):
        if keys[pygame.K_w] or keys[pygame.K_UP]:
            self.target_speed = clamp(self.target_speed + 8.0 * dt, -2.4, self.max_speed)
        elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
            self.target_speed = clamp(self.target_speed - 8.0 * dt, -2.4, self.max_speed)
        else:
            self.target_speed *= max(0.0, 1.0 - 1.5 * dt)

        turn = 0.0
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            turn -= 1.0
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            turn += 1.0

        if abs(turn) > 0.01:
            manual_turn_rate = 95.0 * (0.35 + min(abs(self.speed), 4.0) / 4.0)
            self.heading = (self.heading + turn * manual_turn_rate * dt) % 360

        if keys[pygame.K_SPACE]:
            self.stop()

    def steer_to_target(self, target, dt, final_target=False):
        dx = target[0] - self.x
        dy = target[1] - self.y
        d = math.hypot(dx, dy)
        desired_heading = math.degrees(math.atan2(dy, dx)) % 360
        error = wrap_angle(desired_heading - self.heading)

        self.debug_target = target
        self.debug_error = error
        self.debug_distance = d

        max_turn = self.turn_rate_limit * dt
        turn_step = clamp(error * 3.0 * dt, -max_turn, max_turn)
        self.heading = (self.heading + turn_step) % 360

        abs_error = abs(error)
        heading_factor = clamp(1.0 - abs_error / 140.0, 0.20, 1.0)

        if final_target:
            desired_speed = clamp(d / 22.0, 0.0, 4.8) * heading_factor
            if d < self.final_accept_radius:
                self.target_speed = 0.0
                self.speed *= max(0.0, 1.0 - 5.0 * dt)
                return True
        else:
            desired_speed = clamp(d / 18.0, 1.4, 5.2) * heading_factor
            if d < self.accept_radius:
                return True

        self.target_speed = desired_speed
        return False

    def update_autopilot(self, dt):
        if not self.route:
            self.set_hold()
            return

        if self.wp_index >= len(self.route):
            self.set_hold()
            return

        final_target = self.wp_index == len(self.route) - 1
        target = self.route[self.wp_index]
        reached = self.steer_to_target(target, dt, final_target=final_target)

        if reached:
            self.wp_index += 1
            if self.wp_index >= len(self.route):
                self.set_hold()
                self.hold_position = (self.x, self.y)

    def update_return_home(self, dt):
        target = self.home
        reached = self.steer_to_target(target, dt, final_target=True)
        if reached and abs(self.speed) < 0.35:
            self.x, self.y = self.home
            self.speed = 0.0
            self.target_speed = 0.0
            self.mode = "HOLD"
            self.hold_position = self.home

    def update_hold(self, dt):
        d = distance((self.x, self.y), self.hold_position)
        if d < 8.0:
            self.target_speed = 0.0
            self.speed *= max(0.0, 1.0 - 4.0 * dt)
            return
        self.steer_to_target(self.hold_position, dt, final_target=True)

    def update(self, keys, dt, t):
        if self.mode == "MANUAL":
            self.update_manual(keys, dt)
        elif self.mode == "AUTOPILOT":
            self.update_autopilot(dt)
        elif self.mode == "RETURN":
            self.update_return_home(dt)
        elif self.mode == "HOLD":
            self.update_hold(dt)

        acceleration = 4.2
        self.speed += (self.target_speed - self.speed) * clamp(acceleration * dt, 0, 1)

        cx, cy = self.water_current(t)
        self.current = (cx, cy)

        rad = math.radians(self.heading)
        self.x += math.cos(rad) * self.speed * 13.0 * dt + cx * 20.0 * dt
        self.y += math.sin(rad) * self.speed * 13.0 * dt + cy * 20.0 * dt

        self.x = clamp(self.x, 30.0, WORLD_W - 30.0)
        self.y = clamp(self.y, 30.0, WORLD_H - 30.0)

        normalized = self.target_speed / self.max_speed if self.max_speed else 0
        steer_part = clamp(self.debug_error / 80.0, -1.0, 1.0) if self.mode != "MANUAL" else 0.0
        self.left_motor = clamp(normalized - steer_part * 0.25, -1.0, 1.0)
        self.right_motor = clamp(normalized + steer_part * 0.25, -1.0, 1.0)

        self.depth = self.terrain_depth() + random.uniform(-0.05, 0.05)
        self.temperature = 18.0 + 2.0 * math.sin(self.x / 280.0) + random.uniform(-0.03, 0.03)
        self.roll = 3.5 * math.sin(t * 2.0 + self.x * 0.012)
        self.pitch = 2.5 * math.cos(t * 1.7 + self.y * 0.010)

        dist_home = distance((self.x, self.y), self.home)
        self.link = clamp(100.0 - dist_home / 17.0 + random.uniform(-1.0, 1.0), 5.0, 100.0)

        if not self.rtk_on:
            self.gps_status = "NO RTK"
            self.gps_satellites = 8
        elif self.link < 22:
            self.gps_status = "RTK FLOAT"
            self.gps_satellites = 12
        else:
            self.gps_status = "RTK FIX"
            self.gps_satellites = 18 + int(2 * math.sin(t * 0.7))

        drain = (0.0007 + 0.0015 * abs(self.speed) + (0.0008 if self.sonar_on else 0) + (0.0008 if self.adcp_on else 0)) * dt * 60
        self.battery = max(0.0, self.battery - drain)

        self.track.append((self.x, self.y))
        if len(self.track) > 2500:
            self.track.pop(0)

        if self.recording and self.sonar_on and t - self.last_sample_time >= 0.18:
            self.last_sample_time = t
            cell = (int(self.x // 25), int(self.y // 25))
            if cell not in self.scanned_cells:
                self.scanned_cells.add(cell)
                self.coverage_m2 += 625.0
            self.samples.append({
                "time_s": round(t, 2),
                "x": round(self.x, 2),
                "y": round(self.y, 2),
                "speed_mps": round(abs(self.speed), 3),
                "heading_deg": round(self.heading, 2),
                "depth_m": round(self.depth, 3),
                "temperature_c": round(self.temperature, 2),
                "battery_percent": round(self.battery, 2),
                "gps_status": self.gps_status,
                "current_x_mps": round(cx, 3),
                "current_y_mps": round(cy, 3),
            })


class Simulator:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("APACHE4 — симулятор автономного катера")
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        self.clock = pygame.time.Clock()
        self.fonts = Fonts()
        self.boat = HydroBoat()
        self.log = EventLog()
        self.t = 0.0
        self.wave = 0.0
        self.running = True

        self.survey_rect = pygame.Rect(135, 120, 1030, 535)
        self.route = self.make_lawnmower_route(self.survey_rect, 85)

        self.buttons = []
        self.make_buttons()

        self.log.add("Проект запущен. Управление: W/A/S/D или стрелки.")
        self.log.add("Для демонстрации: «Маршрут» → «Автопилот» → «Экспорт».")
        self.log.add("Автопилот исправлен: стабильный курс, проход точек, возврат домой.")

    def make_lawnmower_route(self, rect, spacing):
        route = []
        y = rect.y + 45
        left = rect.x + 55
        right = rect.right - 55
        direction = 1

        while y <= rect.bottom - 45:
            if direction == 1:
                route.append((left, y))
                route.append((right, y))
            else:
                route.append((right, y))
                route.append((left, y))
            y += spacing
            direction *= -1
        return route

    def make_buttons(self):
        x = SIDE.x + 18
        y = SIDE.y + 526
        w = (SIDE.w - 52) // 2
        h = 35
        gap = 10

        data = [
            ("Ручной", "manual"),
            ("Автопилот", "auto"),
            ("Удержание", "hold"),
            ("Домой", "home"),
            ("Запись", "record"),
            ("Эхолот", "sonar"),
            ("ADCP", "adcp"),
            ("RTK", "rtk"),
            ("Маршрут", "route"),
            ("Экспорт", "export"),
            ("Сброс", "reset"),
            ("Стоп", "stop"),
        ]

        for i, (label, command) in enumerate(data):
            col = i % 2
            row = i // 2
            self.buttons.append(Button((x + col * (w + gap), y + row * (h + gap), w, h), label, command))

    def handle_command(self, command):
        if command == "manual":
            self.boat.set_manual()
            self.log.add("Включён ручной режим.")
        elif command == "auto":
            self.boat.set_autopilot(self.route)
            self.log.add(f"Автопилот запущен. Точек маршрута: {len(self.route)}.")
        elif command == "hold":
            self.boat.set_hold()
            self.log.add("Удержание текущей позиции.")
        elif command == "home":
            self.boat.set_return_home()
            self.log.add("Возврат на базу запущен.")
        elif command == "record":
            self.boat.recording = not self.boat.recording
            self.log.add("Запись измерений: " + ("ВКЛ" if self.boat.recording else "ВЫКЛ"))
        elif command == "sonar":
            self.boat.sonar_on = not self.boat.sonar_on
            self.log.add("Эхолот: " + ("ВКЛ" if self.boat.sonar_on else "ВЫКЛ"))
        elif command == "adcp":
            self.boat.adcp_on = not self.boat.adcp_on
            self.log.add("ADCP: " + ("ВКЛ" if self.boat.adcp_on else "ВЫКЛ"))
        elif command == "rtk":
            self.boat.rtk_on = not self.boat.rtk_on
            self.log.add("RTK GNSS: " + ("ВКЛ" if self.boat.rtk_on else "ВЫКЛ"))
        elif command == "route":
            spacing = random.choice([70, 80, 90, 100])
            self.route = self.make_lawnmower_route(self.survey_rect, spacing)
            self.log.add(f"Построен новый маршрут «змейкой», шаг {spacing}.")
        elif command == "export":
            self.export()
        elif command == "reset":
            self.boat = HydroBoat()
            self.log.add("Симуляция сброшена.")
        elif command == "stop":
            self.boat.stop()
            self.boat.mode = "MANUAL"
            self.log.add("Стоп. Ручной режим.")

    def export(self):
        ensure_outputs()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join("outputs", f"apache_measurements_{stamp}.csv")
        json_path = os.path.join("outputs", f"apache_report_{stamp}.json")

        samples = self.boat.samples
        if samples:
            with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
                writer = csv.DictWriter(f, fieldnames=list(samples[0].keys()))
                writer.writeheader()
                writer.writerows(samples)
        else:
            with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
                writer = csv.writer(f)
                writer.writerow(["note"])
                writer.writerow(["Нет записанных измерений. Включите запись и эхолот."])

        report = {
            "project": "APACHE4 Exam Ready Simulator",
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "mode": self.boat.mode,
            "route_points": len(self.route),
            "completed_waypoints": self.boat.wp_index,
            "track_points": len(self.boat.track),
            "samples_count": len(samples),
            "coverage_m2": round(self.boat.coverage_m2, 1),
            "battery_percent": round(self.boat.battery, 2),
            "last_depth_m": round(self.boat.depth, 2),
            "gps_status": self.boat.gps_status,
            "implemented_functions": [
                "manual_control",
                "autopilot_lawnmower_survey",
                "return_home",
                "position_hold",
                "echo_sounder_depth_recording",
                "adcp_current_vector_simulation",
                "rtk_gnss_status_simulation",
                "telemetry_dashboard",
                "track_map",
                "depth_profile",
                "csv_json_export",
            ],
        }

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)

        self.log.add(f"CSV сохранён: {csv_path}")
        self.log.add(f"JSON отчёт: {json_path}")

    def world_to_view(self, x, y):
        return (
            int(VIEW.x + x / WORLD_W * VIEW.w),
            int(VIEW.y + y / WORLD_H * VIEW.h),
        )

    def world_to_map(self, x, y):
        inner = pygame.Rect(MAP.x + 14, MAP.y + 50, MAP.w - 28, MAP.h - 64)
        return (
            int(inner.x + x / WORLD_W * inner.w),
            int(inner.y + y / WORLD_H * inner.h),
        )

    def draw_panel(self, rect, title):
        pygame.draw.rect(self.screen, PANEL, rect, border_radius=14)
        pygame.draw.rect(self.screen, BORDER, rect, 1, border_radius=14)
        if title:
            self.fonts.draw(self.screen, title, (rect.x + 14, rect.y + 10), WHITE, self.fonts.medium)
            pygame.draw.line(self.screen, BORDER, (rect.x + 12, rect.y + 42), (rect.right - 12, rect.y + 42), 1)

    def draw_bar(self, x, y, w, h, value, color, label):
        pygame.draw.rect(self.screen, (31, 48, 66), (x, y, w, h), border_radius=6)
        pygame.draw.rect(self.screen, color, (x, y, int(w * clamp(value, 0, 1)), h), border_radius=6)
        pygame.draw.rect(self.screen, BORDER, (x, y, w, h), 1, border_radius=6)
        self.fonts.draw(self.screen, label, (x + 6, y + 2), WHITE, self.fonts.small)

    def draw_header(self):
        self.fonts.draw(self.screen, "APACHE4", (24, 18), ORANGE, self.fonts.big)
        self.fonts.draw(self.screen, "десктоп-симулятор автономного сканирования водоёмов", (185, 29), MUTED, self.fonts.medium)

        mode_color = {
            "MANUAL": BLUE,
            "AUTOPILOT": GREEN,
            "RETURN": ORANGE,
            "HOLD": YELLOW,
        }.get(self.boat.mode, WHITE)
        mode_rect = pygame.Rect(VIEW.right - 320, 22, 300, 34)
        pygame.draw.rect(self.screen, PANEL2, mode_rect, border_radius=10)
        self.fonts.draw(self.screen, f"Режим: {self.boat.mode}", mode_rect.center, mode_color, self.fonts.medium, center=True)

    def draw_water(self):
        pygame.draw.rect(self.screen, WATER, VIEW, border_radius=14)
        old_clip = self.screen.get_clip()
        self.screen.set_clip(VIEW)

        for y in range(VIEW.y - 30, VIEW.bottom + 30, 27):
            points = []
            for x in range(VIEW.x - 20, VIEW.right + 20, 18):
                yy = y + math.sin(x * 0.022 + self.wave) * 5
                points.append((x, yy))
            pygame.draw.lines(self.screen, (25, 116, 155), False, points, 1)

        for x in range(VIEW.x, VIEW.right, 70):
            pygame.draw.line(self.screen, (12, 83, 113), (x, VIEW.y), (x, VIEW.bottom), 1)
        for y in range(VIEW.y, VIEW.bottom, 70):
            pygame.draw.line(self.screen, (12, 83, 113), (VIEW.x, y), (VIEW.right, y), 1)

        area_tl = self.world_to_view(self.survey_rect.x, self.survey_rect.y)
        area_size = (
            int(self.survey_rect.w / WORLD_W * VIEW.w),
            int(self.survey_rect.h / WORLD_H * VIEW.h),
        )
        area = pygame.Rect(area_tl, area_size)
        pygame.draw.rect(self.screen, (42, 188, 180), area, 2, border_radius=6)
        self.fonts.draw(self.screen, "зона сканирования", (area.x + 10, area.y + 8), CYAN, self.fonts.small)

        if len(self.route) > 1:
            pts = [self.world_to_view(*p) for p in self.route]
            pygame.draw.lines(self.screen, YELLOW, False, pts, 2)
            for i, p in enumerate(pts):
                color = GREEN if self.boat.mode == "AUTOPILOT" and i == self.boat.wp_index else YELLOW
                pygame.draw.circle(self.screen, color, p, 4)

        if len(self.boat.track) > 1:
            pts = [self.world_to_view(*p) for p in self.boat.track[-1000:]]
            pygame.draw.lines(self.screen, CYAN, False, pts, 2)

        for sample in self.boat.samples[-700::4]:
            px, py = self.world_to_view(sample["x"], sample["y"])
            d = clamp(sample["depth_m"] / 24.0, 0, 1)
            color = (int(80 + d * 100), int(220 - d * 120), int(255 - d * 30))
            pygame.draw.circle(self.screen, color, (px, py), 2)

        home = self.world_to_view(*self.boat.home)
        pygame.draw.circle(self.screen, GREEN, home, 8)
        pygame.draw.circle(self.screen, WHITE, home, 8, 1)
        self.fonts.draw(self.screen, "BASE", (home[0] + 10, home[1] - 9), GREEN, self.fonts.small)

        if self.boat.debug_target and self.boat.mode in ("AUTOPILOT", "RETURN", "HOLD"):
            target = self.world_to_view(*self.boat.debug_target)
            boat = self.world_to_view(self.boat.x, self.boat.y)
            pygame.draw.line(self.screen, GREEN, boat, target, 1)
            pygame.draw.circle(self.screen, GREEN, target, 8, 2)

        self.screen.set_clip(old_clip)
        pygame.draw.rect(self.screen, BORDER, VIEW, 1, border_radius=14)

    def draw_boat(self):
        bx, by = self.world_to_view(self.boat.x, self.boat.y)

        sprite = pygame.Surface((142, 82), pygame.SRCALPHA)
        pygame.draw.ellipse(sprite, (0, 0, 0, 85), (12, 58, 118, 16))
        pygame.draw.polygon(sprite, ORANGE, [(9, 23), (27, 12), (109, 12), (132, 25), (112, 42), (25, 42)])
        pygame.draw.polygon(sprite, ORANGE2, [(17, 51), (31, 41), (106, 41), (125, 52), (105, 66), (32, 66)])
        pygame.draw.rect(sprite, (32, 39, 47), (47, 24, 48, 28), border_radius=9)
        pygame.draw.rect(sprite, (9, 17, 25), (59, 15, 24, 10), border_radius=4)
        pygame.draw.circle(sprite, (18, 25, 33), (101, 29), 6)
        pygame.draw.line(sprite, WHITE, (53, 24), (47, 4), 2)
        pygame.draw.line(sprite, WHITE, (86, 24), (94, 4), 2)
        pygame.draw.polygon(sprite, WHITE, [(132, 44), (114, 34), (114, 54)])

        rotated = pygame.transform.rotate(sprite, -self.boat.heading)
        rect = rotated.get_rect(center=(bx, by))
        self.screen.blit(rotated, rect)

        if self.boat.adcp_on:
            cx, cy = self.boat.current
            pygame.draw.line(self.screen, CYAN, (bx, by), (int(bx + cx * 250), int(by + cy * 250)), 3)
            pygame.draw.circle(self.screen, CYAN, (int(bx + cx * 250), int(by + cy * 250)), 4)

    def draw_side(self):
        self.draw_panel(SIDE, "Телеметрия и управление")
        x = SIDE.x + 18
        y = SIDE.y + 58

        rows = [
            ("Скорость", f"{abs(self.boat.speed):.2f} м/с", GREEN),
            ("Целевая скорость", f"{self.boat.target_speed:.2f} м/с", BLUE),
            ("Курс", f"{self.boat.heading:06.2f}°", CYAN),
            ("Ошибка курса", f"{self.boat.debug_error:+.1f}°", YELLOW),
            ("До цели", f"{self.boat.debug_distance:.1f}", WHITE),
            ("Глубина", f"{self.boat.depth:.2f} м", BLUE),
            ("GPS", f"{self.boat.gps_status} / {self.boat.gps_satellites}", GREEN if self.boat.gps_status == "RTK FIX" else YELLOW),
            ("Связь", f"{self.boat.link:.0f}%", GREEN if self.boat.link > 55 else YELLOW if self.boat.link > 25 else RED),
            ("Крен / дифф.", f"{self.boat.roll:+.1f}° / {self.boat.pitch:+.1f}°", WHITE),
            ("Точек съёмки", str(len(self.boat.samples)), WHITE),
            ("Покрытие", f"{self.boat.coverage_m2:.0f} м²", WHITE),
        ]

        for label, value, color in rows:
            self.fonts.draw(self.screen, label, (x, y), MUTED, self.fonts.small)
            self.fonts.draw(self.screen, value, (x + 145, y - 2), color, self.fonts.normal)
            y += 27

        self.draw_bar(x, y + 5, SIDE.w - 36, 20, self.boat.battery / 100.0,
                      GREEN if self.boat.battery > 35 else YELLOW if self.boat.battery > 15 else RED,
                      f"Батарея: {self.boat.battery:.0f}%")
        y += 36
        self.draw_bar(x, y, SIDE.w - 36, 18, (self.boat.left_motor + 1) / 2,
                      ORANGE, f"Левый двигатель: {self.boat.left_motor * 100:.0f}%")
        y += 24
        self.draw_bar(x, y, SIDE.w - 36, 18, (self.boat.right_motor + 1) / 2,
                      ORANGE, f"Правый двигатель: {self.boat.right_motor * 100:.0f}%")

        y += 38
        flags = [
            ("Эхолот", self.boat.sonar_on),
            ("ADCP", self.boat.adcp_on),
            ("RTK", self.boat.rtk_on),
            ("IMU", self.boat.imu_ok),
            ("Запись", self.boat.recording),
        ]
        for i, (name, state) in enumerate(flags):
            xx = x + (i % 2) * 150
            yy = y + (i // 2) * 23
            pygame.draw.circle(self.screen, GREEN if state else RED, (xx + 8, yy + 9), 6)
            self.fonts.draw(self.screen, name, (xx + 22, yy), WHITE, self.fonts.small)

        for button in self.buttons:
            button.active = (
                (button.command == "manual" and self.boat.mode == "MANUAL")
                or (button.command == "auto" and self.boat.mode == "AUTOPILOT")
                or (button.command == "hold" and self.boat.mode == "HOLD")
                or (button.command == "home" and self.boat.mode == "RETURN")
                or (button.command == "record" and self.boat.recording)
                or (button.command == "sonar" and self.boat.sonar_on)
                or (button.command == "adcp" and self.boat.adcp_on)
                or (button.command == "rtk" and self.boat.rtk_on)
            )
            button.draw(self.screen, self.fonts)

        hy = SIDE.bottom - 82
        self.fonts.draw(self.screen, "Горячие клавиши:", (x, hy), WHITE, self.fonts.small)
        self.fonts.draw(self.screen, "WASD/стрелки — управление, Space — стоп", (x, hy + 20), MUTED, self.fonts.small)
        self.fonts.draw(self.screen, "P — автопилот, H — hold, B — домой", (x, hy + 40), MUTED, self.fonts.small)
        self.fonts.draw(self.screen, "R — запись, E — эхолот, C — ADCP, X — экспорт", (x, hy + 60), MUTED, self.fonts.small)

    def draw_minimap(self):
        self.draw_panel(MAP, "Карта маршрута")
        inner = pygame.Rect(MAP.x + 14, MAP.y + 50, MAP.w - 28, MAP.h - 64)
        pygame.draw.rect(self.screen, (9, 42, 64), inner, border_radius=8)
        pygame.draw.rect(self.screen, BORDER, inner, 1, border_radius=8)

        if len(self.route) > 1:
            pts = [self.world_to_map(*p) for p in self.route]
            pygame.draw.lines(self.screen, YELLOW, False, pts, 2)
            for p in pts:
                pygame.draw.circle(self.screen, YELLOW, p, 3)

        if len(self.boat.track) > 1:
            pts = [self.world_to_map(*p) for p in self.boat.track[-1200:]]
            pygame.draw.lines(self.screen, CYAN, False, pts, 1)

        for sample in self.boat.samples[-280::4]:
            pygame.draw.circle(self.screen, BLUE, self.world_to_map(sample["x"], sample["y"]), 2)

        pygame.draw.circle(self.screen, GREEN, self.world_to_map(*self.boat.home), 5)
        pygame.draw.circle(self.screen, ORANGE, self.world_to_map(self.boat.x, self.boat.y), 8)
        pygame.draw.circle(self.screen, WHITE, self.world_to_map(self.boat.x, self.boat.y), 8, 1)

    def draw_depth_graph(self):
        self.draw_panel(GRAPH, "Профиль глубин")
        inner = pygame.Rect(GRAPH.x + 18, GRAPH.y + 52, GRAPH.w - 36, GRAPH.h - 70)
        pygame.draw.rect(self.screen, (12, 23, 35), inner, border_radius=8)
        pygame.draw.rect(self.screen, BORDER, inner, 1, border_radius=8)

        samples = self.boat.samples[-100:]
        if len(samples) < 2:
            self.fonts.draw(self.screen, "Запусти запись и эхолот.", (inner.x + 14, inner.y + 18), MUTED, self.fonts.small)
            return

        depths = [s["depth_m"] for s in samples]
        d_min = min(depths)
        d_max = max(depths)
        if abs(d_max - d_min) < 0.01:
            d_max += 1.0

        pts = []
        for i, d in enumerate(depths):
            xx = inner.x + int(i / (len(depths) - 1) * inner.w)
            yy = inner.bottom - int((d - d_min) / (d_max - d_min) * inner.h)
            pts.append((xx, yy))

        pygame.draw.lines(self.screen, BLUE, False, pts, 2)
        for p in pts[::10]:
            pygame.draw.circle(self.screen, CYAN, p, 3)

        self.fonts.draw(self.screen, f"min {d_min:.1f} м", (inner.x + 8, inner.bottom - 22), MUTED, self.fonts.small)
        self.fonts.draw(self.screen, f"max {d_max:.1f} м", (inner.x + 8, inner.y + 6), MUTED, self.fonts.small)

    def draw_log(self):
        self.draw_panel(LOGBOX, "Журнал")

        old_clip = self.screen.get_clip()
        self.screen.set_clip(LOGBOX)

        y = LOGBOX.y + 50
        for item in self.log.items:
            self.fonts.draw(self.screen, item, (LOGBOX.x + 14, y), MUTED, self.fonts.small)
            y += 19

        self.screen.set_clip(old_clip)

    def events(self):
        keys = pygame.key.get_pressed()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_p:
                    self.handle_command("auto")
                elif event.key == pygame.K_h:
                    self.handle_command("hold")
                elif event.key == pygame.K_b:
                    self.handle_command("home")
                elif event.key == pygame.K_r:
                    self.handle_command("record")
                elif event.key == pygame.K_e:
                    self.handle_command("sonar")
                elif event.key == pygame.K_c:
                    self.handle_command("adcp")
                elif event.key == pygame.K_g:
                    self.handle_command("rtk")
                elif event.key == pygame.K_m:
                    self.handle_command("route")
                elif event.key == pygame.K_x:
                    self.handle_command("export")
                elif event.key == pygame.K_BACKSPACE:
                    self.handle_command("reset")

            for button in self.buttons:
                if button.clicked(event):
                    self.handle_command(button.command)

        return keys

    def update(self, dt, keys):
        self.t += dt
        self.wave += dt * 2.0
        self.boat.update(keys, dt, self.t)

    def draw(self):
        self.screen.fill(BG)
        self.draw_header()
        self.draw_water()
        self.draw_boat()
        self.draw_side()
        self.draw_minimap()
        self.draw_depth_graph()
        self.draw_log()
        pygame.display.flip()

    def run(self):
        while self.running:
            dt = self.clock.tick(FPS) / 1000.0
            keys = self.events()
            self.update(dt, keys)
            self.draw()
        pygame.quit()


if __name__ == "__main__":
    try:
        Simulator().run()
    except Exception as exc:
        print("Ошибка:", exc)
        raise