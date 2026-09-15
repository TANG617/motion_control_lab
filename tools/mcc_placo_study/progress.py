"""Local-only, throttled progress for per-record evidence verification."""
import time
from evidence import write_json


class SampleProgress:
    def __init__(self, output, stage, total, callback=None):
        self.path = output / 'sample_progress.json'
        self.stage = stage
        self.total = total
        self.callback = callback
        self.started = time.monotonic()
        self.last = -float('inf')
        self.update(0)

    def update(self, current):
        now = time.monotonic()
        if current != self.total and now - self.last < 0.5:
            return
        self.last = now
        record = {'stage': self.stage, 'current': current, 'total': self.total,
                  'elapsed_s': now - self.started}
        write_json(self.path, record)
        if self.callback:
            self.callback(record)
