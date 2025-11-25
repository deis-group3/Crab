import pigpio
import time

ENC_LEFT = 17
ENC_RIGHT = 22

class SingleEncoder:
    def __init__(self, pi, pin, name=""):
        self.pi = pi
        self.pin = pin
        self.name = name
        self.ticks = 0
        self.last_tick_time = time.time()
        self.speed = 0.0

        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_UP)
        self.cb = pi.callback(pin, pigpio.RISING_EDGE, self._pulse)

        self.direction = 1

    def _pulse(self, gpio, level, tick):
        now = time.time()
        dt = now - self.last_tick_time
        self.last_tick_time = now

        self.ticks += 1 * self.direction
        if dt > 0:
            self.speed = 1.0 / dt * self.direction

    def get_ticks(self):
        return self.ticks

    def set_direction(self, dir):
        self.direction = 1 if dir >= 0 else -1

    def get_speed(self):
        return self.speed