"""In-memory store for everything the browser streams back; exposed as pandas DataFrames."""
from __future__ import annotations

import threading

import pandas as pd

TABLES = ("telemetry", "events", "pickups", "trips", "rf", "zn_log")
LIMITS = dict(telemetry=60000, events=5000, pickups=2000, trips=2000, rf=50000, zn_log=20000)


class Store:
    def __init__(self):
        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        with getattr(self, "lock", threading.Lock()):
            self.rows = {t: [] for t in TABLES}
            self.session = None

    def ingest(self, payload: dict):
        with self.lock:
            sid = payload.get("session")
            if sid and sid != self.session:      # a page reload starts a new simulation
                self.rows = {t: [] for t in TABLES}
                self.session = sid
            for t in TABLES:
                rows = payload.get(t) or []
                if rows:
                    self.rows[t].extend(rows)
                    over = len(self.rows[t]) - LIMITS[t]
                    if over > 0:
                        del self.rows[t][:over]
            return {t: len(v) for t, v in self.rows.items()}

    def frames(self) -> dict[str, pd.DataFrame]:
        with self.lock:
            return {t: pd.DataFrame(list(v)) for t, v in self.rows.items()}


STORE = Store()
