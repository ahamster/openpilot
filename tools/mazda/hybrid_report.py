#!/usr/bin/env python3
"""What the hybrid longitudinal did on a drive (Mazda + radar emulation + Hybrid Longitudinal). Read-only: it only reads rlogs.

  python3 tools/mazda/hybrid_report.py                 newest route
  python3 tools/mazda/hybrid_report.py <route>         e.g. 00000074--a469916bb3 (a prefix is enough)
  python3 tools/mazda/hybrid_report.py --last 3        the newest 3 routes
  python3 tools/mazda/hybrid_report.py --quiet         only the summary, no timeline

Timeline: every change of ACC master (stock radar silent / back, openpilot's replacement frames starting / stopping),
the UDS requests openpilot sent the radar, the RES presses it made and the driver's, cruise engaged / dropped,
openpilot engaged / disengaged, experimental mode on / off, the hybrid events openpilot logged (mazdaHybrid*),
canValid dropping, and any alert that names the radar. Then the numbers that matter:
  - takeovers and hand-backs, how long each switch took, and the stock command openpilot started from
  - every stretch with cruise engaged and the radar back in standby after a restart (throttle, nothing braking)
  - every stretch with cruise engaged and no ACC master at all (nobody sending CRZ_INFO)
  - frames where the stock radar and openpilot both sent CRZ_INFO (must be 0)
  - cruise drops within 3 s of a switch
"""
import argparse
import json
import os
import sys
from collections import Counter

from openpilot.tools.lib.logreader import _LogFileReader

try:
  from openpilot.system.hardware.hw import Paths
  DEFAULT_ROOT = Paths.log_root()
except Exception:
  DEFAULT_ROOT = "/data/media/0/realdata"

CRZ_INFO, CRZ_CTRL, CRZ_BTNS, PEDALS, RADAR_UDS, CRZ_EVENTS = 0x21B, 0x21C, 0x09D, 0x165, 0x764, 0x21F
AFTER_S, AFTER_STEP_S = 8.0, 0.5    # what the radar and the car do after each hand-back, sampled
STOCK, OP_ECHO = 0, 128          # src of the stock radar's frames, and of ours coming back from the panda (bus 0)
ALIVE_S = 0.06                   # a 50 Hz CRZ_INFO stream counts as gone after three missed frames (as in hybrid.py)
SWITCH_WINDOW_S = 3.0            # a cruise drop this soon after a switch is blamed on it
MIN_SPAN_S = 0.3
MPH = 2.23694
UDS_NAMES = {(0x10, 0x02): "programming session (silence)", (0x10, 0x01): "default session (restart)", (0x3E, 0x80): "tester present"}


def accel_cmd(dat: bytes):
  """CRZ_INFO.ACCEL_CMD as the panda unpacks it; None for the standby frame (4094: no command)."""
  cmd = ((((dat[2] & 0x3) << 11) | (dat[3] << 3) | (dat[4] >> 5)) - 4096)
  return cmd if -2000 <= cmd <= 2000 else None


def routes_under(root: str):
  routes = {}
  for name in os.listdir(root):
    path = os.path.join(root, name)
    route, _, seg = name.rpartition("--")
    if os.path.isdir(path) and route and seg.isdigit():
      routes.setdefault(route, []).append((int(seg), path))
  return routes


def log_file(seg_path: str):
  for fn in ("rlog.zst", "rlog.bz2", "rlog"):
    if os.path.isfile(os.path.join(seg_path, fn)):
      return os.path.join(seg_path, fn)
  return None


def hybrid_log_event(text: str):
  """The dict openpilot logged with carlog.warning({...}), if this logMessage is one of the mazdaHybrid* events."""
  if "mazdaHybrid" not in text:
    return None
  try:
    rec = json.loads(text)
  except ValueError:
    return None
  stack = [rec]
  while stack:
    x = stack.pop()
    if isinstance(x, dict):
      if str(x.get("event", "")).startswith("mazdaHybrid"):
        return x
      stack.extend(x.values())
    elif isinstance(x, str) and x.startswith("{"):
      try:
        stack.append(json.loads(x))
      except ValueError:
        pass
  return None


class Scan:
  def __init__(self):
    self.t0 = None
    self.t = 0.0
    self.commit = ""
    self.v = 0.0
    self.cruise = False          # PEDALS.ACC_ACTIVE, the car's own word for MRCC engaged
    self.op_enabled = False
    self.experimental = False
    self.can_valid = True
    self.alert = ""
    self.stock_last = -1e9       # t of the last stock CRZ_INFO
    self.stock_cmd = None        # its ACCEL_CMD (None = standby)
    self.stock_active = False    # CRZ_CTRL.CRZ_ACTIVE from the stock radar
    self.op_last = -1e9          # t of the last CRZ_INFO openpilot put on bus 0
    self.stock_alive = False
    self.op_active = False
    self.last_switch = None      # (t, kind)
    self.pending_request = None  # (t, kind) of the last session request without an outcome yet
    self.timeline = []
    self.takeovers = []
    self.handbacks = []
    self.double_master = 0
    self.spans = {"standby": [], "no_master": []}   # [start, end, vmin, vmax, ended_by]
    self.open = {"standby": None, "no_master": None}
    self.cruise_drops_after_switch = []
    self.op_presses = 0
    self.press_open = None
    self.log_events = []
    self.radar_alerts = []
    self.can_invalid = []
    self.can_invalid_open = None
    self.duration = 0.0
    # what the panda ran and refused (counters are cumulative since panda boot: first and last seen).
    # The safety mode is counted per message: the panda is in noOutput before CarParams and after the
    # car turns off, so the most common mode is the one that drove.
    self.panda_modes: Counter = Counter()
    self.tx_blocked, self.rx_invalid = [None, None], [None, None]
    self.sent = {"crz_info": 0, "uds": 0, "res": 0}    # what openpilot asked pandad to send (sendcan)
    self.echo = {"crz_info": 0, "uds": 0, "res": 0}    # what came back from the panda as put on bus 0
    # the radar's and the car's state after each hand-back: does MRCC ever drive again, and what does it need
    self.stock_available = False   # CRZ_CTRL.CRZ_AVAILABLE from the stock radar
    self.acc_off = False           # PEDALS.ACC_OFF: MRCC armed but not controlling
    self.set_speed = None          # CRZ_EVENTS.CRZ_SPEED (kph), the dash set speed
    self.crz_started = False       # CRZ_EVENTS.CRZ_STARTED
    self.driver_presses = 0        # RES / SET presses on the wheel (frames)
    self.after = []                # [(t_handback, [samples])]
    self.after_next_t = None

  def note(self, text: str):
    self.timeline.append((self.t, self.v, text))

  def feed(self, evt):
    w = evt.which()
    if self.t0 is None:
      self.t0 = evt.logMonoTime * 1e-9
    self.t = evt.logMonoTime * 1e-9 - self.t0
    self.duration = self.t
    if w == "initData":
      self.commit = evt.initData.gitCommit[:7]
    elif w == "carState":
      cs = evt.carState
      self.v = cs.vEgo
      if self.after and self.t - self.after[-1][0] <= AFTER_S and self.t >= self.after_next_t:
        self.after_next_t = self.t + AFTER_STEP_S
        self.after[-1][1].append((self.t - self.after[-1][0], self.v, self.stock_alive, self.stock_cmd, self.stock_active,
                                  self.stock_available, self.cruise, self.acc_off, self.set_speed, self.crz_started,
                                  self.op_presses, self.driver_presses, self.op_enabled))
      if cs.canValid != self.can_valid:
        self.can_valid = cs.canValid
        if not cs.canValid:
          self.can_invalid_open = self.t
          self.note("canValid FALSE (Unknown Vehicle Variant risk)")
        elif self.can_invalid_open is not None:
          self.can_invalid.append((self.can_invalid_open, self.t))
          self.can_invalid_open = None
          self.note("canValid back")
    elif w == "carControl":
      en = evt.carControl.enabled
      if en != self.op_enabled:
        self.op_enabled = en
        self.note("openpilot engaged" if en else "openpilot disengaged")
    elif w == "selfdriveState":
      ss = evt.selfdriveState
      if ss.experimentalMode != self.experimental:
        self.experimental = ss.experimentalMode
        self.note("experimental mode ON" if ss.experimentalMode else "experimental mode OFF (standard)")
      text = (ss.alertText1 + " " + ss.alertText2).strip()
      if text != self.alert:
        self.alert = text
        low = text.lower()
        if "radar" in low or "unknown vehicle" in low or "can error" in low or "front sensing" in low:
          self.radar_alerts.append((self.t, text))
          self.note(f"ALERT: {text}")
    elif w == "pandaStates":
      for ps in evt.pandaStates:
        self.panda_modes[(str(ps.safetyModel), int(ps.safetyParam))] += 1
        for pair, val in ((self.tx_blocked, ps.safetyTxBlocked), (self.rx_invalid, ps.safetyRxInvalid)):
          if pair[0] is None:
            pair[0] = val
          pair[1] = val
        break
    elif w == "sendcan":
      for c in evt.sendcan:
        if c.src == 0 and c.address == CRZ_INFO:
          self.sent["crz_info"] += 1
        elif c.src == 0 and c.address == RADAR_UDS and len(c.dat) >= 2 and c.dat[0] == 2 and c.dat[1] in (0x10, 0x3E):
          self.sent["uds"] += 1
        elif c.src == 0 and c.address == CRZ_BTNS and (c.dat[0] & 0x04):
          self.sent["res"] += 1
    elif w == "logMessage":
      ev = hybrid_log_event(evt.logMessage)
      if ev is not None:
        self.log_events.append((self.t, ev))
        detail = ", ".join(f"{k}={v}" for k, v in ev.items() if k != "event")
        self.note(f"log {ev['event']}  {detail}")
    elif w == "can":
      for c in evt.can:
        d = c.dat
        if c.src == STOCK and c.address == CRZ_INFO and len(d) >= 8:
          self.stock_last = self.t
          cmd = accel_cmd(d)
          if (cmd is None) != (self.stock_cmd is None) and self.stock_alive:
            self.note("stock radar in standby (no command)" if cmd is None else f"stock radar commanding ({cmd})")
          self.stock_cmd = cmd
        elif c.src == STOCK and c.address == CRZ_CTRL and len(d) >= 8:
          self.stock_active = bool(d[0] & 0x08)
          self.stock_available = bool(d[2] & 0x02)
        elif c.src == STOCK and c.address == CRZ_EVENTS and len(d) >= 8:
          self.set_speed = ((d[0] << 8) | d[1]) * 0.005 - 0.5
          self.crz_started = bool(d[2] & 0x04)
        elif c.src == OP_ECHO and c.address == CRZ_INFO:
          self.op_last = self.t
          self.echo["crz_info"] += 1
          if self.t - self.stock_last < 0.04:
            self.double_master += 1
        elif c.src == OP_ECHO and c.address == RADAR_UDS and len(d) >= 3 and d[0] == 2 and d[1] in (0x10, 0x3E):
          self.echo["uds"] += 1
          kind = UDS_NAMES.get((d[1], d[2]), f"UDS {d[1]:02x} {d[2]:02x}")
          if d[1] == 0x10:
            self.note(f"openpilot -> radar: {kind}")
            self.pending_request = (self.t, kind)
        elif c.src == OP_ECHO and c.address == CRZ_BTNS and (d[0] & 0x04):
          self.op_presses += 1
          self.echo["res"] += 1
          if self.press_open is None or self.t - self.press_open > 0.5:
            self.note("openpilot pressed RES")
          self.press_open = self.t
        elif c.src == STOCK and c.address == CRZ_BTNS and (d[0] & 0x34):
          self.driver_presses += 1
          which = "RES" if d[0] & 0x04 else ("SET+" if d[0] & 0x10 else "SET-")
          if self.press_open is None or self.t - self.press_open > 0.5:
            self.note(f"driver pressed {which}")
          self.press_open = self.t
        elif c.src == STOCK and c.address == PEDALS and len(d) >= 1:
          cruise = bool(d[0] & 0x08)
          self.acc_off = bool(d[0] & 0x04)
          if cruise != self.cruise:
            self.cruise = cruise
            self.note("cruise ENGAGED (car)" if cruise else "cruise DROPPED (car)")
            if not cruise and self.last_switch is not None and self.t - self.last_switch[0] <= SWITCH_WINDOW_S:
              self.cruise_drops_after_switch.append((self.t, self.last_switch[1], self.t - self.last_switch[0]))
    self.update_masters()

  def update_masters(self):
    stock_alive = self.t - self.stock_last < ALIVE_S
    op_active = self.t - self.op_last < ALIVE_S
    if stock_alive != self.stock_alive:
      self.stock_alive = stock_alive
      if not stock_alive:
        self.note("stock radar SILENT")
      else:
        self.note("stock radar BACK")
        if self.pending_request and "restart" in self.pending_request[1]:
          self.handbacks.append((self.t, self.v, self.t - self.pending_request[0]))
          self.last_switch = (self.t, "hand-back")
          self.pending_request = None
          self.after.append((self.t, []))
          self.after_next_t = self.t
    if op_active != self.op_active:
      self.op_active = op_active
      if op_active:
        self.note(f"openpilot replacement frames START (stock last seen {self.t - self.stock_last:.2f} s ago, stock cmd was {self.stock_cmd})")
        if self.pending_request and "silence" in self.pending_request[1]:
          self.takeovers.append((self.t, self.v, self.t - self.pending_request[0], self.stock_cmd))
          self.last_switch = (self.t, "takeover")
          self.pending_request = None
      else:
        self.note("openpilot replacement frames STOP")
    moving = self.v > 0.5
    self.span("standby", self.cruise and stock_alive and self.stock_cmd is None and not op_active and moving)
    self.span("no_master", self.cruise and not stock_alive and not op_active and moving)

  def span(self, kind: str, active: bool):
    cur = self.open[kind]
    if active and cur is None:
      self.open[kind] = [self.t, self.t, self.v, self.v]
    elif active:
      cur[1], cur[2], cur[3] = self.t, min(cur[2], self.v), max(cur[3], self.v)
    elif cur is not None:
      how = "cruise dropped" if not self.cruise else ("openpilot took over" if self.op_active else
                                                       ("stock radar commanding" if self.stock_cmd is not None else "stopped"))
      self.spans[kind].append((cur[0], self.t, cur[2], cur[3], how))
      self.open[kind] = None

  def finish(self):
    for kind in self.open:
      self.span(kind, False)


def scan_route(name: str, segs: list) -> Scan:
  s = Scan()
  for i, seg in enumerate(segs, 1):
    print(f"\rreading {name} segment {i}/{len(segs)}", end="", file=sys.stderr, flush=True)
    fn = log_file(seg)
    if fn is None:
      continue
    try:
      for evt in _LogFileReader(fn):
        s.feed(evt)
    except Exception as e:
      print(f"\n  (skipped {os.path.basename(seg)}: {e})", file=sys.stderr)
  print(file=sys.stderr)
  s.finish()
  return s


def print_report(name: str, s: Scan, quiet: bool):
  print(f"\n=== route {name}  build {s.commit}  {s.duration / 60:.1f} min")
  print(f"  takeovers {len(s.takeovers)}  hand-backs {len(s.handbacks)}  RES frames pressed by openpilot {s.op_presses}" +
        f"  double-master frames {s.double_master}  canValid drops {len(s.can_invalid)}  radar alerts {len(s.radar_alerts)}")
  blocked = (s.tx_blocked[1] - s.tx_blocked[0]) if s.tx_blocked[0] is not None else None
  invalid = (s.rx_invalid[1] - s.rx_invalid[0]) if s.rx_invalid[0] is not None else None
  modes = ", ".join(f"{m} param {prm} ({n} msgs)" for (m, prm), n in s.panda_modes.most_common(3)) or "unknown"
  print(f"  panda safety while driving: {modes}   [265 = GEN1+TI+radar emulation; 9 = GEN1+TI only]")
  print(f"  panda counters over the route: tx blocked {blocked}  rx invalid {invalid}")
  print("  openpilot -> bus: " + ", ".join(f"{k} sent {s.sent[k]} / on the bus {s.echo[k]}" for k in ("crz_info", "uds", "res")) +
        "   (uds = session control and tester present only; sent but not on the bus = refused by the panda)")
  for (t, v, dur, cmd) in s.takeovers:
    print(f"    takeover  at {t:7.1f}s {v * MPH:3.0f} mph: radar silent {dur:.2f} s after the request; started from stock command {cmd}")
  for (t, v, dur), (_, samples) in zip(s.handbacks, s.after, strict=True):
    print(f"    hand-back at {t:7.1f}s {v * MPH:3.0f} mph: radar back {dur:.2f} s after the request")
    print("        +s   mph  radar  stockCmd  CRZ_ACTIVE CRZ_AVAIL  car:cruise accOff  setKph started  RES/SET frames op/driver  op")
    for (dt, vv, alive, cmd, act, avail, cruise, off, spd, started, opp, drv, en) in samples:
      cmd_txt = f"{cmd:6d}" if cmd is not None else "standby"
      spd_txt = f"{spd:6.1f}" if spd is not None else "     ?"
      print(f"      {dt:4.1f} {vv * MPH:5.0f}  {'alive' if alive else 'quiet':>5}  {cmd_txt:>8}  {int(act):10d} {int(avail):9d}" +
            f"  {int(cruise):10d} {int(off):6d}  {spd_txt} {int(started):7d}  {opp:8d} / {drv:<6d}  {'on' if en else 'off'}")
  for kind, title in (("standby", "cruise engaged, radar alive but in standby, openpilot not driving (throttle, no brakes)"),
                      ("no_master", "cruise engaged and nobody sending CRZ_INFO")):
    long_ones = [x for x in s.spans[kind] if x[1] - x[0] >= MIN_SPAN_S]
    total = sum(b - a for a, b, *_ in s.spans[kind])
    print(f"  {title}: {total:.1f} s in total, {len(long_ones)} stretch(es) of {MIN_SPAN_S} s or more")
    for (a, b, vmin, vmax, how) in long_ones:
      print(f"    {a:7.1f}s  {b - a:5.1f} s  {vmin * MPH:3.0f}-{vmax * MPH:<3.0f} mph  ended by: {how}")
  if s.cruise_drops_after_switch:
    print("  cruise drops within 3 s of a switch:")
    for (t, kind, dt) in s.cruise_drops_after_switch:
      print(f"    {t:7.1f}s  {dt:.1f} s after the {kind}")
  if s.radar_alerts:
    print("  alerts naming the radar:")
    for (t, text) in s.radar_alerts:
      print(f"    {t:7.1f}s  {text}")
  if s.can_invalid:
    print("  canValid false: " + ", ".join(f"{a:.1f}-{b:.1f}s" for a, b in s.can_invalid))
  stuck = [e for _, e in s.log_events if e.get("event") in ("mazdaHybridMrccStuck", "mazdaHybridTakeoverRejected",
                                                             "mazdaHybridSilenceFailed", "mazdaHybridRestoreFailed", "mazdaHybridRadarReturned")]
  if stuck:
    print("  hybrid faults logged by openpilot: " + ", ".join(e["event"] for e in stuck))
  if not quiet:
    print("  timeline:")
    for (t, v, text) in s.timeline:
      print(f"    {t:7.1f}s {v * MPH:3.0f} mph  {text}")


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route", nargs="?")
  ap.add_argument("--root", default=DEFAULT_ROOT)
  ap.add_argument("--last", type=int, default=1, help="newest N routes (ignored when a route is given)")
  ap.add_argument("--quiet", action="store_true", help="summary only")
  args = ap.parse_args()
  if not os.path.isdir(args.root):
    sys.exit(f"no log folder at {args.root} (pass --root)")
  routes = routes_under(args.root)
  if not routes:
    sys.exit(f"no routes under {args.root}")
  if args.route:
    names = [n for n in routes if n == args.route or n.startswith(args.route) or n.endswith(args.route)]
    if not names:
      sys.exit(f"route {args.route} not found under {args.root}")
  else:
    names = sorted(routes, key=lambda n: os.path.getmtime(max(routes[n])[1]))[-args.last:]
  for name in names:
    segs = [p for _, p in sorted(routes[name])]
    print_report(name, scan_route(name, segs), args.quiet)


if __name__ == "__main__":
  main()
