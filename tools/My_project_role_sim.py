#!/usr/bin/env python3
"""
Offline test bench for the planning / state-machine part of navigator.py
(Workstream C: plan_from, FOLLOW -> BACKTRACK -> REPLAN -> FOLLOW).

Runs on plain Python (Windows, macOS or Linux) WITHOUT ROS or Gazebo:
  * rclpy and the ROS message packages are replaced by tiny stand-ins,
  * the robot is a simple unicycle model (x, y, yaw) driven by /cmd_vel,
  * perception is a TEST DOUBLE: a list of edges this script pretends are blocked.

This file is a test tool only. It never goes into navigator.py, and the
blocked-edge list below must never be copied into navigator.py.

Usage (from the repository root):
    python tools/My_project_role_sim.py vmc2_project/src/navigator.py
    python tools/My_project_role_sim.py vmc2_project/src/navigator.py --stub-control
    python tools/My_project_role_sim.py vmc2_project/src/navigator.py --only C

--stub-control  use a built-in simple controller instead of your turtlebot_control()
                (handy while the controller is not finished yet)
--only X        run only scenario X (A, B, C, D or E)
--own-perception keep your own is_passable() (for example to test the
                AMAZE_NAV_FAKE_BLOCKED hook; there is no LiDAR in this test bench)
--quiet         print only the summary lines
"""
import argparse
import importlib.util
import math
import os
import sys
import types

# Test 1: Stand-ins for ROS, so that navigator.py can be imported without ROS
SIM = {"t": 0.0, "cmd": (0.0, 0.0), "log": [], "quiet": False}


class _Logger:
    def _p(self, lvl, msg):
        line = f"[{SIM['t']:7.2f}s] {lvl:5s} {msg}"
        SIM["log"].append(line)
        if not SIM["quiet"]:
            print("    " + line)

    def info(self, msg, *a, **k): self._p("INFO", msg)
    def warn(self, msg, *a, **k): self._p("WARN", msg)
    warning = warn
    def error(self, msg, *a, **k): self._p("ERROR", msg)
    def debug(self, msg, *a, **k): pass


class _Time:
    def __init__(self, t): self.nanoseconds = int(t * 1e9)


class _Clock:
    def now(self): return _Time(SIM["t"])


class _Pub:
    def publish(self, msg): SIM["cmd"] = (float(msg.linear.x), float(msg.angular.z))


class _Node:
    def __init__(self, name, *a, **k): self._name = name
    def get_logger(self): return _Logger()
    def get_clock(self): return _Clock()
    def create_publisher(self, *a, **k): return _Pub()
    def create_subscription(self, *a, **k): return None
    def create_timer(self, *a, **k): return None
    def destroy_node(self): pass


class _Vec:
    def __init__(self): self.x = 0.0; self.y = 0.0; self.z = 0.0


class _Twist:
    def __init__(self): self.linear = _Vec(); self.angular = _Vec()


def _install_ros_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m
    mod("rclpy", init=lambda *a, **k: None, spin=lambda *a, **k: None,
        shutdown=lambda *a, **k: None)
    mod("rclpy.node", Node=_Node)
    mod("rclpy.parameter", Parameter=object)
    mod("rcl_interfaces"); mod("rcl_interfaces.msg", SetParametersResult=object)
    mod("geometry_msgs"); mod("geometry_msgs.msg", Twist=_Twist)
    mod("nav_msgs"); mod("nav_msgs.msg", Odometry=object)
    mod("sensor_msgs"); mod("sensor_msgs.msg", LaserScan=object)
    try:
        import cv2  # real OpenCV if installed; only the window calls are disabled
    except ImportError:
        cv2 = mod("cv2")
    for name in ("namedWindow", "imshow", "destroyAllWindows"):
        setattr(cv2, name, lambda *a, **k: None)
    cv2.waitKey = lambda *a, **k: -1


# 2) The maze graph and the test double for perception
XS = [0.712, 2.136, 3.560, 4.984, 6.408]
YS = [0.0, -1.424, -2.848, -4.272, -5.696]
NODE = {(round(XS[c], 3), round(YS[r], 3)): (r, c) for r in range(5) for c in range(5)}

# Blocked edges measured from the track mesh (project_track.dae). TEST DATA ONLY.
MAZE_BLOCKED = ["0,1-0,2", "1,0-1,1", "1,1-1,2", "1,3-1,4", "2,1-2,2",
                "3,2-3,3", "4,2-4,3", "0,3-1,3", "2,0-3,0"]


# Corridor centre lines: all 40 edges plus the entry and exit corridors.
CENTRE_LINES = [((-0.3, 0.0), (0.712, 0.0)), ((6.408, -5.696), (7.5, -5.696))]
CENTRE_LINES += [((XS[c], YS[r]), (XS[c + 1], YS[r])) for r in range(5) for c in range(4)]
CENTRE_LINES += [((XS[c], YS[r]), (XS[c], YS[r + 1])) for r in range(4) for c in range(5)]


def rc(p):
    """(x, y) -> (r, c) if p is a graph node, else None."""
    return NODE.get((round(p[0], 3), round(p[1], 3)))


def name(p):
    i = rc(p)
    if i is not None:
        return f"({i[0]},{i[1]})"
    return f"({p[0]:.3f},{p[1]:.3f})"


def parse_edges(spec):
    out = set()
    for e in spec:
        a, b = e.split("-")
        a = tuple(int(v) for v in a.split(","))
        b = tuple(int(v) for v in b.split(","))
        out.add(frozenset((a, b)))
    return out


def make_fake_is_passable(blocked, detect_at):
    """
    is_passable() test double. An edge in `blocked` is reported blocked once the
    robot is within `detect_at` metres of the middle of that edge (the real LiDAR
    sees about 1 m). Before that, and for every other edge, it returns True.
    """
    def is_passable(self, seg_dir_yaw):
        _, seg_idx, _ = nav.nearest_point_on_polyline(self.full_path, (self.x, self.y))
        ga, gb = self.full_path[seg_idx], self.full_path[seg_idx + 1]
        a, b = rc(ga), rc(gb)
        if a is None or b is None or frozenset((a, b)) not in blocked:
            return True
        mid = ((ga[0] + gb[0]) / 2, (ga[1] + gb[1]) / 2)
        return math.dist((self.x, self.y), mid) > detect_at
    return is_passable


def stage_a_target(self, robot_pos, seg_dir_yaw):
    """obstacle_avoided_target() stand-in: plain lookahead on the current segment."""
    _, seg_idx, _ = nav.nearest_point_on_polyline(self.full_path, robot_pos)
    seg = [self.full_path[seg_idx], self.full_path[seg_idx + 1]]
    return nav.lookahead_point(seg, robot_pos, 0.35)


def stub_control(self, robot_pos, robot_h, target, dt, Ld):
    """Simple rotate-then-drive controller (used with --stub-control)."""
    dx, dy = target[0] - robot_pos[0], target[1] - robot_pos[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-3:
        return 0.0, 0.0
    alpha = nav.angle_wrap(math.atan2(dy, dx) - robot_h)
    w = max(-2.0, min(2.0, 2.5 * alpha))
    if abs(alpha) > math.radians(30):
        return 0.0, w
    v = 0.22 * math.cos(alpha) * min(1.0, dist / 0.15)
    return max(v, 0.03), w


def _is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


# Test 3: One scenario = one run of the navigator from a start pose
def run(label, start, blocked_spec, detect_at, stub, own_perception=False, max_t=400.0):
    SIM["t"] = 0.0; SIM["cmd"] = (0.0, 0.0); SIM["log"] = []
    env = os.environ.get("AMAZE_NAV_FAKE_BLOCKED", "").replace(" ", "")
    if own_perception and env:
        blocked_spec = [e for e in env.split(";") if e]  # judge removals against the hook's list
    blocked = parse_edges(blocked_spec)
    cls = nav.GridPathFollowerNode
    saved = {k: cls.__dict__[k] for k in
             ("is_passable", "obstacle_avoided_target", "turtlebot_control")}
    if not own_perception:
        cls.is_passable = make_fake_is_passable(blocked, detect_at)
    cls.obstacle_avoided_target = stage_a_target
    if stub:
        cls.turtlebot_control = stub_control

    # Teleport rule: start_pos must equal the pose the robot starts from.
    # path_plan() is called at the end of __init__ with self.start_pos.
    orig_path_plan = cls.path_plan
    def path_plan(self, start_from=None):
        if getattr(self, "_first_plan_done", False) is False:
            self._first_plan_done = True
            self.start_pos = start
            start_from = start
        return orig_path_plan(self, start_from)
    cls.path_plan = path_plan

    modes, removed, problems = [], [], []
    try:
        node = cls()
    finally:
        cls.path_plan = orig_path_plan

    # Fill template placeholders (...) so the run does not crash.
    if not _is_number(getattr(node, "lookahead_dist", None)):
        node.lookahead_dist = 0.35
    if not _is_number(getattr(node, "v_max", None)):
        node.v_max = 0.22
    if not _is_number(getattr(node, "w_max", None)):
        node.w_max = 2.0
    pid = getattr(node, "pid_angular", None)
    if pid is None or not _is_number(getattr(pid, "kp", None)):
        node.pid_angular = nav.PID(kp=1.5, ki=0.0, kd=0.05, i_limit=0.5)

    # Record mode changes and edge removals.
    orig_set_mode = node._set_mode
    def set_mode(new_mode):
        if node.mode != new_mode:
            modes.append((SIM["t"], new_mode))
        orig_set_mode(new_mode)
    node._set_mode = set_mode
    orig_remove = node.graph.remove_edge
    def remove_edge(a, b):
        removed.append((name(a), name(b), rc(a), rc(b)))
        orig_remove(a, b)
    node.graph.remove_edge = remove_edge

    first_path = [name(p) for p in node.full_path]
    x, y, yaw = start[0], start[1], 0.0
    # face along the first edge, like a teleport with the right heading
    if len(node.full_path) >= 2:
        a, b = node.full_path[0], node.full_path[1]
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    node.x, node.y, node.yaw, node.pose_ready = x, y, yaw, True
    visited, max_off, dt = [], 0.0, 0.05
    replan_starts = []

    while SIM["t"] < max_t:
        SIM["t"] += dt
        before = node.mode
        try:
            node.control_step()
        except Exception as exc:  # report the first crash and stop this run
            problems.append(f"control_step() raised {type(exc).__name__}: {exc}")
            break
        if before == "REPLAN" and node.mode == "FOLLOW":
            replan_starts.append((node.full_path[0] if node.full_path else None,
                                  node.prev_node))
        if node.mode == "DONE":
            break
        v, w = SIM["cmd"]
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = nav.angle_wrap(yaw + w * dt)
        node.x, node.y, node.yaw = x, y, yaw
        n = min(NODE, key=lambda p: math.dist(p, (x, y)))
        if math.dist(n, (x, y)) < 0.06 and (not visited or visited[-1] != NODE[n]):
            visited.append(NODE[n])
        off = min(math.dist(nav.nearest_point_on_polyline(list(s), (x, y))[2], (x, y))
                  for s in CENTRE_LINES)
        max_off = max(max_off, off)

    for k, v in saved.items():
        setattr(cls, k, v)

    # ---------------- checks (the Definition of Done of Workstream C) ----------
    done = node.mode == "DONE"
    rem_sets = [frozenset((r[2], r[3])) for r in removed]
    wrong = [r for r, s in zip(removed, rem_sets) if s not in blocked]
    dup = len(rem_sets) != len(set(rem_sets))
    bt = sum(1 for _, m in modes if m == "BACKTRACK")
    bad_start = [p for p in replan_starts if p[1] is not None and p[0] != p[1]]
    checks = [
        ("goal reached (mode DONE)", done),
        ("only edges from the blocked list were removed", not wrong),
        ("no edge removed twice / no BACKTRACK<->REPLAN loop",
         not dup and bt <= len(removed) + (0 if done else 1)),
        ("each new plan starts at prev_node", not bad_start),
        ("robot stayed near the corridor centre lines (< 0.25 m)", max_off < 0.25),
        ("no crash in control_step()", not problems),
    ]
    print(f"\n=== Scenario {label} ===")
    print(f"  start {start}, blocked test edges: {len(blocked)}, detect at <= {detect_at} m from edge middle")
    print(f"  first plan : {' -> '.join(first_path)}")
    for r in removed:
        print(f"  removed    : {r[0]} <-> {r[1]}")
    print(f"  nodes seen : {' '.join(f'({a},{b})' for a, b in visited)}")
    print(f"  sim time   : {SIM['t']:.1f} s (kinematic model, Gazebo will be slower)")
    print(f"  max offset : {max_off:.3f} m from a corridor centre line")
    for p in problems:
        print(f"  PROBLEM    : {p}")
    ok_all = True
    for text, ok in checks:
        ok_all &= ok
        print(f"  [{'PASS' if ok else 'FAIL'}] {text}")
    return ok_all


# Test 4: Unit tests for plan_from()
def unit_tests():
    print("=== Unit tests: plan_from() ===")
    SIM["quiet"] = True
    node = nav.GridPathFollowerNode()
    g = node.graph
    P = {}  # plans, computed lazily so one crash does not hide the other tests

    def plan(key, start):
        if key not in P:
            P[key] = node.plan_from(start)
        return P[key]

    def cut_goal_and_plan():
        # cut the goal node off: (4,4) has neighbours (3,4) and (4,3)
        g.remove_edge(g.coord_of((3, 4)), g.coord_of((4, 4)))
        g.remove_edge(g.coord_of((4, 3)), g.coord_of((4, 4)))
        return node.plan_from((3.560, -4.272))

    S = (0.0, 0.0)
    tests = [
        ("from (0,0): first point is the start pose",
         lambda: plan("s", S)[:1] == [S]),
        ("from (0,0): second point is node (0,0)",
         lambda: rc(plan("s", S)[1]) == (0, 0)),
        ("from (0,0): last point is the goal (7.12,-5.696)",
         lambda: math.dist(plan("s", S)[-1], (7.120, -5.696)) < 1e-6),
        ("from (0,0): 11 waypoints with all edges free",
         lambda: len(plan("s", S)) == 11),
        ("consecutive nodes are graph neighbours",
         lambda: len(plan("s", S)) > 3 and all(g.has_edge(a, b) for a, b in
                                               zip(plan("s", S)[1:-1], plan("s", S)[2:-1]))),
        ("no duplicate consecutive points",
         lambda: all(math.dist(a, b) > 1e-6 for a, b in zip(plan("s", S), plan("s", S)[1:]))),
        ("from node (3,2): path starts exactly at the node, no duplicate",
         lambda: plan("n", (3.560, -4.272))[0] == (3.560, -4.272)
         and math.dist(plan("n", (3.560, -4.272))[0], plan("n", (3.560, -4.272))[1]) > 1e-6),
        ("no route: returns [start_pose] only (robot stays still)",
         lambda: cut_goal_and_plan() == [(3.560, -4.272)]),
    ]
    results = []
    for text, fn in tests:
        try:
            ok = bool(fn())
        except Exception as exc:
            ok = False
            text += f"   <- {type(exc).__name__}: {exc}"
        results.append((text, ok))
    for text, ok in results:
        print(f"  [{'PASS' if ok else 'FAIL'}] {text}")
    SIM["quiet"] = False
    return all(ok for _, ok in results)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("navigator")
    ap.add_argument("--stub-control", action="store_true")
    ap.add_argument("--only", default="")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--own-perception", action="store_true")
    args = ap.parse_args()

    _install_ros_stubs()
    spec = importlib.util.spec_from_file_location("navigator_under_test", args.navigator)
    nav = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(nav)

    scenarios = {
        "A": ("no obstacles, full run", (0.0, 0.0), [], 0.75),
        "B": ("one blocked edge, from checkpoint 1", (3.560, -4.272), ["3,2-3,3"], 0.75),
        "C": ("real maze blockages, full run", (0.0, 0.0), MAZE_BLOCKED, 0.75),
        "D": ("real maze, blockage seen late (mid-edge backtrack)", (0.0, 0.0), MAZE_BLOCKED, 0.35),
        "E": ("real maze, from checkpoint 2", (4.984, -4.272), MAZE_BLOCKED, 0.75),
    }
    all_ok = unit_tests()
    SIM["quiet"] = args.quiet
    for key, (title, start, blk, det) in scenarios.items():
        if args.only and key not in args.only.upper():
            continue
        all_ok &= run(f"{key}: {title}", start, blk, det, args.stub_control, args.own_perception)
    print("\nOVERALL:", "PASS" if all_ok else "FAIL")
    sys.exit(0 if all_ok else 1)
