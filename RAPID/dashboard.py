"""Dashboard of the robots (importances, current tasks, knowledge of every robot).

Simulator side : `Dashboard.tick()` is called by `Environment.run()`, builds every `period` steps a small snapshot of
native python types (`collect_snapshot`) and sends it with `put_nowait` to a separate process that draws it with
matplotlib. If the dashboard is late, snapshots are dropped, the simulator is never blocked.
The dashboard only reads the simulation, it never writes in the env, the robots or the belief spaces.

`import RAPID.dashboard` does not import matplotlib, only the child process does.
Scripts using `display_dashboard=True` need an `if __name__ == "__main__":` guard (the child is started with `spawn`).

Keys in the window : left/right (or 1, 2, 3) switch the view : importances, tasks, knowledge (one map per robot),
p pause the display, n normalise the alphas by their sum, t show the true positions of the believed robots.
"""
import os
import sys
import zlib
import math
import queue
import multiprocessing
from collections import deque

import numpy as np

VIEW_MODES = ["importances", "tasks", "knowledge"]  # keys 1-3, left/right cycle
MAX_PATH_POINTS = 500
HISTORY_LEN = 2000
TYPE_PALETTE = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b", "#e377c2", "#17becf", "#bcbd22", "#7f7f7f"]
ANSI_PALETTE = [34, 33, 32, 31, 35, 36, 91, 92, 93, 94]
BAR_CHARS = " ▏▎▍▌▋▊▉█"


# ----------------------------------------------------------------------------------------------------------------
# Snapshot (simulator side)
# ----------------------------------------------------------------------------------------------------------------
def _xy(p):
    return (float(p[0]), float(p[1]))


def _is_base_station(robot):
    return any(c.__name__ == "BaseStation" for c in type(robot).__mro__)


def collect_snapshot(env, mode="importances", show_base_stations=False, last_grid_crcs=None):
    """
    Read-only snapshot of the simulation, made of native python types only (picklable, small, no live reference).
    In the knowledge view, the knowledge of every robot is collected; the grid of a robot is only given if its crc
    differs from the one in `last_grid_crcs` ({robot_id: crc}), else it is None.
    """
    robots = []
    for r in env.agents:
        base = _is_base_station(r)
        if base and not show_base_stations:
            continue
        task = None
        act = r.action_to_perform
        if act is not None:
            coords = act.get("coordinates", r.target)
            task = {"type": str(act["type"]),
                    "target": None if coords is None else (int(coords[0]), int(coords[1])),
                    "artifact_id": act.get("id")}
        confidence = None
        if env.step > 0:
            try:
                confidence = float(r.confidence_estimation())
            except Exception:
                confidence = None
        robots.append({"id": int(r.robot_id),
                       "kind": type(r).__name__,
                       "pos": _xy((r.transform.x, r.transform.y)),
                       "status": str(r.status),
                       "imdone": bool(r.imdone),
                       "importances": {str(t): float(c["importance"]) for t, c in r.competences.items()},
                       "capabilities": {str(t): float(c["capability"]) for t, c in r.competences.items()},
                       "task": task,
                       "confidence": confidence})
    snap = {"step": int(env.step), "robots": robots, "knowledge": None, "grid_crcs": {}, "communication_mode": env.communication_mode}
    if mode == "knowledge":
        last = last_grid_crcs or {}
        snap["knowledge"] = {}
        for r in robots:
            k, crc = _collect_knowledge(env, r["id"], last.get(r["id"]))
            if k is not None:
                snap["knowledge"][r["id"]] = k
                snap["grid_crcs"][r["id"]] = crc
    return snap


def _collect_knowledge(env, robot_id, last_grid_crc):
    robot = next((r for r in env.agents if r.robot_id == robot_id), None)
    if robot is None:
        return None, last_grid_crc
    bs = robot.belief_space
    true_pos = {a.robot_id: _xy((a.transform.x, a.transform.y)) for a in env.agents}
    others = []
    for rid, info in bs["robot_informations"].items():
        if rid == robot.robot_id:
            continue
        others.append({"id": int(rid), "believed_pos": _xy(info["position"]), "belief_step": int(info["step"]),
                       "true_pos": true_pos.get(rid)})
    artifacts = [{"id": a_id, "type": str(a["type"]), "coords": (int(a["coordinates"][0]), int(a["coordinates"][1])),
                  "status": str(a["status"]), "belief_step": int(a["step"])} for a_id, a in bs["artifacts"].items()]
    grid8 = np.ascontiguousarray(bs["occupancy_grid"].astype(np.int8))
    crc = zlib.crc32(grid8)
    path = robot.path_to_target
    last_infos = bs["last_infos_matrix"].get(robot.robot_id, {})
    know = {"robot_id": int(robot.robot_id),
            "grid": grid8 if crc != last_grid_crc else None,
            "grid_shape": grid8.shape,
            "known_cells": int(np.count_nonzero(grid8 != -1)),
            "others": others,
            "artifacts": artifacts,
            "target": None if robot.target is None else (int(robot.target[0]), int(robot.target[1])),
            "path": None if path is None else [(int(p[0]), int(p[1])) for p in list(path)[:MAX_PATH_POINTS]],
            "vision_range": float(robot.vision_range),
            "communication_range": float(robot.communication_range),
            "last_infos": {int(k): int(v) for k, v in last_infos.items()}}
    return know, crc


# ----------------------------------------------------------------------------------------------------------------
# Terminal mode
# ----------------------------------------------------------------------------------------------------------------
def _type_index(types_seen, t):
    if t not in types_seen:
        types_seen[t] = len(types_seen)
    return types_seen[t]


def _bar(value, vmax, width):
    n = max(0.0, width * value / vmax)
    full = int(n)
    frac = int(round((n - full) * 8))
    if frac == 8:
        full, frac = full + 1, 0
    return (BAR_CHARS[8] * full + (BAR_CHARS[frac] if frac else "")).ljust(width + 1)


def render_importances_text(snapshot, width=24, color=None, types_seen=None):
    """One line per (robot, task type) with an unicode bar of the importance alpha, and the sum of alphas per robot."""
    if color is None:
        color = sys.stdout.isatty()
    if types_seen is None:
        types_seen = {}
    robots = snapshot["robots"]
    vmax = max([1e-9] + [v for r in robots for v in r["importances"].values()])
    name_w = max([len(t) for r in robots for t in r["importances"]] + [1])
    lines = [f"step {snapshot['step']} " + "─" * 40]
    for r in robots:
        cur = r["task"]["type"] if r["task"] else None
        for i, (t, a) in enumerate(r["importances"].items()):
            bar = _bar(a, vmax, width)
            if color:
                bar = f"\x1b[{ANSI_PALETTE[_type_index(types_seen, t) % len(ANSI_PALETTE)]}m{bar}\x1b[0m"
            head = f"R{r['id']}" if i == 0 else ""
            mark = f"   ◀ tâche: {cur}" if t == cur else ""
            lines.append(f"{head:<4}{t:<{name_w}}  {bar} {a:6.3f}{mark}")
        lines.append(f"    Σα = {sum(r['importances'].values()):.3f}" + ("   (done)" if r["imdone"] else ""))
    return "\n".join(lines)


# ----------------------------------------------------------------------------------------------------------------
# Dashboard (simulator side)
# ----------------------------------------------------------------------------------------------------------------
class Dashboard:
    def __init__(self, env, mode="importances", period=10):
        self.env = env
        self.mode = mode
        self.period = max(1, int(period))
        self.enabled = True
        self.proc = None
        self.view_mode = mode if mode in VIEW_MODES else "importances"
        self.last_grid_crcs = {}
        self.types_seen = {}
        self.dry_run = bool(os.environ.get("RAPID_DASHBOARD_DRY_RUN"))  # the child only consumes the queue (tests)

        if mode != "terminal" and not self.dry_run:
            headless = os.environ.get("MPLBACKEND", "").lower() == "agg" or (
                sys.platform.startswith("linux") and not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"))
            if headless:
                print("dashboard : no display available, falling back to the 'terminal' mode.")
                self.mode = "terminal"
        if self.mode != "terminal":
            try:
                ctx = multiprocessing.get_context("spawn")
                self.q_snap = ctx.Queue(maxsize=1)
                self.q_cmd = ctx.Queue()
                config = {"mode": self.view_mode, "dry_run": self.dry_run,
                          "title": f"RAPID dashboard ({env.communication_mode})"}
                self.proc = ctx.Process(target=_dashboard_process, args=(self.q_snap, self.q_cmd, config), daemon=True)
                self.proc.start()
            except Exception as e:
                self._disable(f"could not start the dashboard process ({e!r})")

    def _disable(self, why):
        if self.enabled:
            print(f"dashboard disabled : {why}")
        self.enabled = False

    def tick(self):
        if not self.enabled or self.env.step % self.period:
            return
        try:
            if self.mode == "terminal":
                snap = collect_snapshot(self.env)
                print("\x1b[H\x1b[J" + render_importances_text(snap, types_seen=self.types_seen), flush=True)
                return
            if self.proc is None or not self.proc.is_alive():
                self._disable("the dashboard window is closed")
                return
            self._read_commands()
            snap = collect_snapshot(self.env, self.view_mode, last_grid_crcs=self.last_grid_crcs)
            try:
                self.q_snap.put_nowait(snap)
            except queue.Full:
                return  # the dashboard is late : this image is dropped (and the grid will be sent again)
            self.last_grid_crcs.update(snap["grid_crcs"])
        except Exception as e:
            self._disable(repr(e))

    def _read_commands(self):
        while True:
            try:
                cmd = self.q_cmd.get_nowait()
            except queue.Empty:
                return
            if "mode" in cmd and cmd["mode"] in VIEW_MODES:
                self.view_mode = cmd["mode"]
                self.last_grid_crcs = {}

    def close(self):
        if self.proc is None:
            return
        try:
            self.q_snap.put(None, timeout=0.2)
        except Exception:
            pass
        try:
            self.proc.join(timeout=1)
            if self.proc.is_alive():
                self.proc.terminate()
        except Exception:
            pass
        self.enabled = False


# ----------------------------------------------------------------------------------------------------------------
# Window process (matplotlib)
# ----------------------------------------------------------------------------------------------------------------
class _State:
    """Everything the window process remembers."""
    def __init__(self, config):
        self.mode = config["mode"]
        self.paused = False
        self.normalize = False
        self.show_true = False
        self.snap = None
        self.history = deque(maxlen=HISTORY_LEN)  # (step, {rid: {type: alpha}}, {rid: task type or None})
        self.type_colors = {}
        self.grids = {}  # robot_id -> last received grid
        self.view = {}    # artists of the current layout
        self.layout_key = None

    def color(self, t):
        if t not in self.type_colors:
            self.type_colors[t] = TYPE_PALETTE[len(self.type_colors) % len(TYPE_PALETTE)]
        return self.type_colors[t]

    def robot_ids(self):
        return [r["id"] for r in self.snap["robots"]] if self.snap else []

    def types(self):
        seen = []
        for r in (self.snap["robots"] if self.snap else []):
            for t in r["importances"]:
                if t not in seen:
                    seen.append(t)
        return seen


def _update_state(st, snap):
    st.snap = snap
    st.history.append((snap["step"], {r["id"]: dict(r["importances"]) for r in snap["robots"]},
                       {r["id"]: (r["task"]["type"] if r["task"] else None) for r in snap["robots"]}))
    for rid, k in (snap.get("knowledge") or {}).items():
        if k["grid"] is not None:
            st.grids[rid] = k["grid"]


def _grid_shape(n):
    cols = math.ceil(math.sqrt(n))
    return math.ceil(n / cols), cols


def _layout(fig, st):
    """(Re)creates the axes of the current mode. Called only when the mode or the set of robots/types changes."""
    fig.clf()
    st.view = {}
    ids = st.robot_ids()
    if st.mode == "importances":
        gs = fig.add_gridspec(1, 2, width_ratios=[1, 1.4])
        st.view["bar"] = fig.add_subplot(gs[0, 0])
        rows, cols = _grid_shape(len(ids))
        sub = gs[0, 1].subgridspec(rows, cols)
        st.view["lines_ax"] = {rid: fig.add_subplot(sub[i // cols, i % cols]) for i, rid in enumerate(ids)}
    elif st.mode == "tasks":
        st.view["text"] = fig.add_subplot(2, 1, 1)
        st.view["stack"] = fig.add_subplot(2, 1, 2)
    else:
        rows, cols = _grid_shape(len(ids))
        gs = fig.add_gridspec(rows, cols)
        st.view["maps"] = {rid: {"ax": fig.add_subplot(gs[i // cols, i % cols])} for i, rid in enumerate(ids)}
    st.layout_key = (st.mode, tuple(ids), tuple(st.types()))
    fig.tight_layout()


def _draw_importances(st):
    v, snap = st.view, st.snap
    robots, types = snap["robots"], st.types()
    ax = v["bar"]
    n = len(types)
    w = 0.8 / max(1, n)
    if "bars" not in v:
        v["bars"] = {}
        for j, t in enumerate(types):
            xs = np.arange(len(robots)) + (j - (n - 1) / 2) * w
            v["bars"][t] = ax.bar(xs, [0] * len(robots), w, color=st.color(t), label=t)
        ax.set_xticks(range(len(robots)), [f"R{r['id']}" for r in robots])
        ax.legend(loc="upper right", fontsize=8)
        ax.set_ylabel("alpha (normalised by sum)" if st.normalize else "alpha")
    top, bottom = 0.1, 0.0
    for t in types:
        for rect, r in zip(v["bars"][t], robots):
            a = r["importances"].get(t, 0.0)
            if st.normalize:
                a = a / max(1e-9, sum(r["importances"].values()))
            rect.set_height(a)
            top, bottom = max(top, a), min(bottom, a)
            current = r["task"] is not None and r["task"]["type"] == t
            rect.set_hatch("//" if current else None)
            rect.set_edgecolor("k" if current else rect.get_facecolor())
            rect.set_alpha(0.3 if r["imdone"] else 1.0)
    ax.set_ylim(bottom * 1.1, top * 1.1)
    ax.set_title(f"importances (hatched = current task) - step {snap['step']}", fontsize=9)
    _draw_alpha_history(st)


def _draw_alpha_history(st):
    """alpha(t) of every robot, one small axis per robot."""
    v = st.view
    types = st.types()
    steps = [h[0] for h in st.history]
    if "lines" not in v:
        v["lines"] = {}
        for rid, ax in v["lines_ax"].items():
            v["lines"][rid] = {t: ax.plot([], [], color=st.color(t), label=t)[0] for t in types}
            ax.set_title(f"R{rid}", fontsize=8)
            ax.tick_params(labelsize=6)
        next(iter(v["lines_ax"].values())).legend(loc="upper left", fontsize=6)
    for rid, ax in v["lines_ax"].items():
        top, bottom = 0.1, 0.0
        for t, line in v["lines"][rid].items():
            ys = [h[1].get(rid, {}).get(t, np.nan) for h in st.history]
            if st.normalize:
                ys = [y / max(1e-9, sum(h[1].get(rid, {}).values())) for y, h in zip(ys, st.history)]
            line.set_data(steps, ys)
            if len(ys) and not np.all(np.isnan(ys)):
                top, bottom = max(top, np.nanmax(ys)), min(bottom, np.nanmin(ys))
        ax.set_xlim(steps[0], max(steps[-1], steps[0] + 1))
        ax.set_ylim(bottom * 1.1, top * 1.1)


def _draw_tasks(st):
    v, snap = st.view, st.snap
    lines = [f"{'robot':<7}{'status':<12}{'task':<16}{'target':<12}{'dist':>6}{'sum a':>8}"]
    for r in snap["robots"]:
        t = r["task"]
        tgt = "-" if t is None or t["target"] is None else f"{t['target'][0]},{t['target'][1]}"
        dist = "-" if t is None or t["target"] is None else f"{math.hypot(r['pos'][0] - t['target'][0], r['pos'][1] - t['target'][1]):.1f}"
        lines.append(f"{'R' + str(r['id']) + (' (done)' if r['imdone'] else ''):<7}{r['status'][:11]:<12}{(t['type'] if t else '-')[:15]:<16}"
                     f"{tgt:<12}{dist:>6}{sum(r['importances'].values()):>8.3f}")
    ax = v["text"]
    if "txt" not in v:
        ax.axis("off")
        v["txt"] = ax.text(0.0, 1.0, "", va="top", ha="left", family="monospace", fontsize=9, transform=ax.transAxes)
    v["txt"].set_text("\n".join(lines))
    if "stack" in v:
        ax = v["stack"]
        ax.clear()
        task_types = sorted({tt for h in st.history for tt in h[2].values() if tt})
        if task_types:
            steps = [h[0] for h in st.history]
            counts = [[sum(1 for x in h[2].values() if x == tt) for h in st.history] for tt in task_types]
            ax.stackplot(steps, counts, labels=task_types, colors=[st.color(tt) for tt in task_types])
            ax.legend(loc="upper left", fontsize=8)
        ax.set_xlabel("step")
        ax.set_ylabel("robots per task")


_CELL_COLORS = {-1: "#555555", 0: "#ffffff", 1: "#000000", 11: "#cc0000", 2: "#3060e0", 3: "#e8d060", 4: "#40a040", 100: "#ffffff"}


def _grid_to_rgb(grid):
    lut = np.ones((130, 3))
    from matplotlib.colors import to_rgb
    for val, c in _CELL_COLORS.items():
        lut[val + 1] = to_rgb(c)
    return lut[grid.astype(np.intp) + 1].transpose(1, 0, 2)  # the grid is indexed [x, y] : transposed for imshow


def _draw_knowledge(st):
    """One map per robot : its own belief of the grid, of the other robots (alpha = age) and of the artifacts."""
    from matplotlib.patches import Circle
    from matplotlib.lines import Line2D
    snap = st.snap
    knowledge = snap.get("knowledge") or {}
    for rid, v in st.view["maps"].items():
        ax, k = v["ax"], knowledge.get(rid)
        if k is None or rid not in st.grids:
            ax.set_title(f"R{rid} : waiting...", fontsize=8)
            continue
        grid = st.grids[rid]
        if "im" not in v:
            v["im"] = ax.imshow(_grid_to_rgb(grid), origin="upper", interpolation="nearest")
            v["self"] = ax.scatter([], [], s=40, c="magenta", edgecolors="k", zorder=5)
            v["vis"] = Circle((0, 0), 1, fill=False, ec="magenta", lw=1)
            v["com"] = Circle((0, 0), 1, fill=False, ec="orange", lw=1, ls="--")
            ax.add_patch(v["vis"]), ax.add_patch(v["com"])
            v["others"] = ax.scatter([], [], s=20, zorder=4, edgecolors="k")
            v["arts"] = ax.scatter([], [], s=25, marker="s", zorder=3, edgecolors="k")
            v["target"] = ax.scatter([], [], s=40, marker="x", c="red", zorder=6)
            v["path"] = Line2D([], [], color="red", lw=1)
            ax.add_line(v["path"])
            v["true"] = ax.plot([], [], ls=":", color="gray", lw=1)[0]
            ax.set_xlim(-0.5, grid.shape[0] - 0.5)
            ax.set_ylim(grid.shape[1] - 0.5, -0.5)
            ax.tick_params(labelsize=6)
        if k["grid"] is not None:
            v["im"].set_data(_grid_to_rgb(grid))
        me = next((r for r in snap["robots"] if r["id"] == rid), None)
        if me is not None:
            v["self"].set_offsets([me["pos"]])
            for c, key in ((v["vis"], "vision_range"), (v["com"], "communication_range")):
                c.set_center(me["pos"]), c.set_radius(k[key])
        others = k["others"]
        if others:
            v["others"].set_offsets([o["believed_pos"] for o in others])
            v["others"].set_facecolors([(0.1, 0.4, 1.0, max(0.1, 1.0 - (snap["step"] - o["belief_step"]) / 200.0)) for o in others])
        else:
            v["others"].set_offsets(np.empty((0, 2)))
        arts = k["artifacts"]
        if arts:
            v["arts"].set_offsets([a["coords"] for a in arts])
            v["arts"].set_facecolors(["#40d040" if a["status"] == "done" else "#ff9020" for a in arts])
            v["arts"].set_edgecolors([st.color(a["type"]) for a in arts])
        else:
            v["arts"].set_offsets(np.empty((0, 2)))
        v["target"].set_offsets([k["target"]] if k["target"] else np.empty((0, 2)))
        if k["path"]:
            v["path"].set_data([p[0] for p in k["path"]], [p[1] for p in k["path"]])
        else:
            v["path"].set_data([], [])
        xs, ys = [], []
        if st.show_true:
            for o in others:
                if o["true_pos"] is not None:
                    xs += [o["believed_pos"][0], o["true_pos"][0], np.nan]
                    ys += [o["believed_pos"][1], o["true_pos"][1], np.nan]
        v["true"].set_data(xs, ys)
        total = grid.shape[0] * grid.shape[1]
        conf = "-" if me is None or me["confidence"] is None else f"{me['confidence']:.2f}"
        ax.set_title(f"R{rid}  known {100 * k['known_cells'] / total:.0f}%  conf {conf}", fontsize=8)


def _redraw(fig, st):
    if st.snap is None:
        return
    key = (st.mode, tuple(st.robot_ids()), tuple(st.types()))
    if key != st.layout_key:
        _layout(fig, st)
    if "bar" in st.view:
        _draw_importances(st)
    if "text" in st.view:
        _draw_tasks(st)
    if "maps" in st.view:
        _draw_knowledge(st)
    fig.suptitle(st.snap and f"{'[paused] ' if st.paused else ''}step {st.snap['step']}", fontsize=8)
    fig.canvas.draw_idle()


def _dashboard_process(q_snap, q_cmd, config):
    """Target of the window process : gets snapshots, redraws, sends back the commands (mode, selected robot)."""
    if config.get("dry_run"):
        while q_snap.get() is not None:
            pass
        return
    import matplotlib
    if not os.environ.get("MPLBACKEND"):
        for backend in ("TkAgg", "QtAgg"):
            try:
                matplotlib.use(backend)
                break
            except Exception:
                continue
    import matplotlib.pyplot as plt
    plt.ion()
    fig = plt.figure(figsize=(12, 6))
    try:
        fig.canvas.manager.set_window_title(config.get("title", "RAPID dashboard"))
    except Exception:
        pass
    st = _State(config)

    def send(cmd):
        try:
            q_cmd.put_nowait(cmd)
        except Exception:
            pass

    def on_key(event):
        if event.key in ("1", "2", "3"):
            st.mode = VIEW_MODES[int(event.key) - 1]
        elif event.key in ("left", "right"):
            st.mode = VIEW_MODES[(VIEW_MODES.index(st.mode) + (1 if event.key == "right" else -1)) % len(VIEW_MODES)]
        elif event.key == "p":
            st.paused = not st.paused
        elif event.key == "n":
            st.normalize = not st.normalize
            st.layout_key = None
        elif event.key == "t":
            st.show_true = not st.show_true
        else:
            return
        send({"mode": st.mode})
        _redraw(fig, st)

    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.show(block=False)
    while plt.fignum_exists(fig.number):
        dirty = False
        try:
            snap = q_snap.get(timeout=0.05)
            if snap is None:
                break
            _update_state(st, snap)
            dirty = not st.paused
        except queue.Empty:
            pass
        if dirty:
            _redraw(fig, st)
        fig.canvas.flush_events()
    plt.close("all")
