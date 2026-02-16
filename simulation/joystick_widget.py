"""
Virtual Joystick Widgets for Manual Drone Control

Renders two on-screen joystick circles that the pilot can drag with
the mouse. Left stick controls movement (forward/back/strafe), right
stick controls yaw rotation. Releasing the mouse springs the stick
back to center (0, 0).
"""

import pygame
import math
from typing import Tuple, Optional


class VirtualJoystick:
    """Single virtual joystick — a draggable knob inside a circle."""

    RADIUS = 40        # Base circle radius in pixels
    KNOB_RADIUS = 12   # Draggable knob radius

    def __init__(self, center_x: int, center_y: int, label: str = "",
                 axis_labels: Tuple[str, str, str, str] = ("", "", "", "")):
        self.cx = center_x
        self.cy = center_y
        self.label = label
        # (top, bottom, left, right)
        self.axis_labels = axis_labels

        # Normalized axes: -1 to +1
        self.x = 0.0
        self.y = 0.0

        self._dragging = False

    @property
    def rect(self) -> pygame.Rect:
        return pygame.Rect(self.cx - self.RADIUS, self.cy - self.RADIUS,
                           self.RADIUS * 2, self.RADIUS * 2)

    def handle_mouse_down(self, mx: int, my: int) -> bool:
        """Start drag if click is inside the base circle. Returns True if captured."""
        dist = math.hypot(mx - self.cx, my - self.cy)
        if dist <= self.RADIUS:
            self._dragging = True
            self._update_from_mouse(mx, my)
            return True
        return False

    def handle_mouse_motion(self, mx: int, my: int):
        """Update stick position during drag."""
        if self._dragging:
            self._update_from_mouse(mx, my)

    def handle_mouse_up(self):
        """Release — spring back to center."""
        self._dragging = False
        self.x = 0.0
        self.y = 0.0

    def _update_from_mouse(self, mx: int, my: int):
        dx = mx - self.cx
        dy = my - self.cy
        dist = math.hypot(dx, dy)
        if dist > self.RADIUS:
            dx = dx / dist * self.RADIUS
            dy = dy / dist * self.RADIUS
        self.x = dx / self.RADIUS   # -1 to +1
        self.y = -dy / self.RADIUS  # -1 (down) to +1 (up), inverted for screen coords

    def draw(self, surface: pygame.Surface):
        """Draw the joystick on the given surface."""
        # Base circle
        pygame.draw.circle(surface, (40, 40, 40), (self.cx, self.cy), self.RADIUS, 0)
        pygame.draw.circle(surface, (80, 80, 80), (self.cx, self.cy), self.RADIUS, 2)

        # Crosshair
        pygame.draw.line(surface, (60, 60, 60),
                         (self.cx - self.RADIUS + 5, self.cy),
                         (self.cx + self.RADIUS - 5, self.cy), 1)
        pygame.draw.line(surface, (60, 60, 60),
                         (self.cx, self.cy - self.RADIUS + 5),
                         (self.cx, self.cy + self.RADIUS - 5), 1)

        # Knob position
        knob_x = self.cx + int(self.x * self.RADIUS)
        knob_y = self.cy - int(self.y * self.RADIUS)  # invert Y back to screen
        color = (255, 255, 0) if self._dragging else (200, 200, 0)
        pygame.draw.circle(surface, color, (knob_x, knob_y), self.KNOB_RADIUS)
        pygame.draw.circle(surface, (255, 255, 255), (knob_x, knob_y), self.KNOB_RADIUS, 1)

        # Label below
        font = pygame.font.Font(None, 16)
        if self.label:
            lbl = font.render(self.label, True, (200, 200, 200))
            surface.blit(lbl, (self.cx - lbl.get_width() // 2, self.cy + self.RADIUS + 4))

        # Axis labels (tiny, around the circle)
        tiny = pygame.font.Font(None, 13)
        top, bottom, left, right = self.axis_labels
        if top:
            s = tiny.render(top, True, (150, 150, 150))
            surface.blit(s, (self.cx - s.get_width() // 2, self.cy - self.RADIUS - 12))
        if bottom:
            s = tiny.render(bottom, True, (150, 150, 150))
            surface.blit(s, (self.cx - s.get_width() // 2, self.cy + self.RADIUS + 16))
        if left:
            s = tiny.render(left, True, (150, 150, 150))
            surface.blit(s, (self.cx - self.RADIUS - s.get_width() - 3, self.cy - 5))
        if right:
            s = tiny.render(right, True, (150, 150, 150))
            surface.blit(s, (self.cx + self.RADIUS + 3, self.cy - 5))


class JoystickPanel:
    """Two joysticks side by side — left for movement, right for yaw."""

    WIDTH = 240
    HEIGHT = 120

    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y
        pad = 20
        stick_y = y + self.HEIGHT // 2
        self.left_stick = VirtualJoystick(
            x + pad + VirtualJoystick.RADIUS, stick_y,
            label="Move",
            axis_labels=("Fwd", "Back", "L", "R"))
        self.right_stick = VirtualJoystick(
            x + self.WIDTH - pad - VirtualJoystick.RADIUS, stick_y,
            label="Yaw",
            axis_labels=("", "", "L", "R"))

    @property
    def rect(self) -> pygame.Rect:
        return pygame.Rect(self.x, self.y, self.WIDTH, self.HEIGHT)

    def handle_mouse_down(self, mx: int, my: int) -> bool:
        """Try both sticks. Returns True if either captured the click."""
        if self.left_stick.handle_mouse_down(mx, my):
            return True
        if self.right_stick.handle_mouse_down(mx, my):
            return True
        return False

    def handle_mouse_motion(self, mx: int, my: int):
        self.left_stick.handle_mouse_motion(mx, my)
        self.right_stick.handle_mouse_motion(mx, my)

    def handle_mouse_up(self):
        self.left_stick.handle_mouse_up()
        self.right_stick.handle_mouse_up()

    def get_axes(self) -> Tuple[float, float, float, float]:
        """Return (left_x, left_y, right_x, right_y) all in [-1, +1]."""
        return (self.left_stick.x, self.left_stick.y,
                self.right_stick.x, self.right_stick.y)

    def draw(self, surface: pygame.Surface):
        """Draw panel background + both sticks."""
        # Semi-transparent background
        bg = pygame.Surface((self.WIDTH, self.HEIGHT), pygame.SRCALPHA)
        bg.fill((0, 0, 0, 180))
        surface.blit(bg, (self.x, self.y))
        pygame.draw.rect(surface, (255, 255, 0), (self.x, self.y, self.WIDTH, self.HEIGHT), 1)

        self.left_stick.draw(surface)
        self.right_stick.draw(surface)
