import bisect
import re
from collections import defaultdict
from typing import Dict, List, Optional, Tuple, Set

import carla

# Recorder "world" sentinel
WORLD_REC_ID = 0xFFFFFFFF  # 4294967295

# CARLA prints very small values (e.g. a near-zero roll) in scientific notation, like
# "8.24296e-05" - a plain "-?[0-9.]+" pattern stops at the "e" and the whole line fails to
# match, silently dropping that actor. Use this everywhere a recorder-dump float is parsed.
_FLOAT = r"-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?"


class RecorderIndex:
    """
    Minimal index built from show_recorder_file_info(..., True):
      - frames, duration
      - frame_times[i] = time (s) of recorder Frame (i+1)
      - creates_by_recid[rec_id] = (type_id, create_loc, role_name, created_frame)
      - collisions_by_frame[frame] = [(rec_id_a, rec_id_b), ...]
      - positions_by_frame[frame][rec_id] = (location, rotation), from each frame's "Positions:" block
      - velocities_by_frame[frame][rec_id] = (linear_velocity, angular_velocity), from each
        frame's "Dynamic actors:" block. Only present when the recording was started with
        additional_data=True (client.start_recorder(name, True)); ground-truth physics
        velocity, not derived from position deltas.
    """

    def __init__(self):
        self.frames: int = 0
        self.duration: float = 0.0
        self.frame_times: List[float] = []
        self.creates_by_recid: Dict[int, Tuple[str, carla.Location, Optional[str], int]] = {}
        self.collisions_by_frame: Dict[int, List[Tuple[int, int]]] = defaultdict(list)
        self.positions_by_frame: Dict[int, Dict[int, Tuple[carla.Location, carla.Rotation]]] = defaultdict(dict)
        self.velocities_by_frame: Dict[int, Dict[int, Tuple[carla.Vector3D, carla.Vector3D]]] = defaultdict(dict)

    @staticmethod
    def parse(info: str) -> "RecorderIndex":
        idx = RecorderIndex()

        # Frames / Duration
        m_frames = re.search(r"^Frames:\s+(\d+)", info, re.MULTILINE)
        idx.frames = int(m_frames.group(1)) if m_frames else 0
        m_dur = re.search(rf"^Duration:\s+({_FLOAT})\s+seconds", info, re.MULTILINE)
        idx.duration = float(m_dur.group(1)) if m_dur else 0.0

        # Patterns
        frame_hdr = re.compile(rf"^Frame\s+(\d+)\s+at\s+({_FLOAT})\s+seconds$")
        create_re = re.compile(
            rf"^\s*Create\s+(\d+):\s+([A-Za-z0-9_.]+)\s+\(\d+\)\s+at\s+\(({_FLOAT}),\s*({_FLOAT}),\s*({_FLOAT})\)\s*$"
        )
        role_re = re.compile(r"^\s*role_name\s*=\s*(.+)$")
        # allow optional "(hero)" (or similar) after both IDs
        coll_re = re.compile(
            r"^\s*Collision\s+id\s+(?:\d+)\s+between\s+(\d+)(?:\s+\([^)]*\))?\s+with\s+(\d+)(?:\s+\([^)]*\))?\s*$"
        )
        # "  Id: 47 Location: (9865.4, 2708.58, -0.589614) Rotation: (-0.000457765, 0.363694, 90.9712)"
        # NOTE: the printed Rotation triple is (roll, pitch, yaw), NOT (pitch, yaw, roll) -
        # verified against live actor.get_transform().rotation for a known frame/actor.
        pos_re = re.compile(
            rf"^\s*Id:\s+(\d+)\s+Location:\s+\(({_FLOAT}),\s*({_FLOAT}),\s*({_FLOAT})\)\s+"
            rf"Rotation:\s+\(({_FLOAT}),\s*({_FLOAT}),\s*({_FLOAT})\)\s*$"
        )
        # "  Id: 47 linear_velocity: (-0.0973904, 13.1679, -3.26157e-06) angular_velocity: (-0.0787992, -0.000707775, -0.00361435)"
        # Only present when the recording was made with additional_data=True.
        dyn_re = re.compile(
            rf"^\s*Id:\s+(\d+)\s+linear_velocity:\s+\(({_FLOAT}),\s*({_FLOAT}),\s*({_FLOAT})\)\s+"
            rf"angular_velocity:\s+\(({_FLOAT}),\s*({_FLOAT}),\s*({_FLOAT})\)\s*$"
        )

        lines = info.splitlines()
        curr_frame: Optional[int] = None
        last_create_id: Optional[int] = None
        times_tmp: Dict[int, float] = {}

        for line in lines:
            s = line.strip()

            m = frame_hdr.match(s)
            if m:
                curr_frame = int(m.group(1))
                times_tmp[curr_frame] = float(m.group(2))
                last_create_id = None
                continue

            m = create_re.match(line)
            if m and curr_frame is not None:
                rec_id = int(m.group(1))
                type_id = m.group(2)
                x, y, z = float(m.group(3)), float(m.group(4)), float(m.group(5))
                idx.creates_by_recid[rec_id] = (type_id, carla.Location(x=x, y=y, z=z), None, curr_frame)
                last_create_id = rec_id
                continue

            if last_create_id is not None and line.startswith("  "):
                rr = role_re.match(s)
                if rr:
                    t, loc, _, cf = idx.creates_by_recid[last_create_id]
                    idx.creates_by_recid[last_create_id] = (t, loc, rr.group(1).strip(), cf)
                continue
            else:
                last_create_id = None

            m = coll_re.match(line)
            if m and curr_frame is not None:
                a, b = int(m.group(1)), int(m.group(2))
                idx.collisions_by_frame[curr_frame].append((a, b))
                continue

            m = pos_re.match(s)
            if m and curr_frame is not None:
                rec_id = int(m.group(1))
                lx, ly, lz = float(m.group(2)), float(m.group(3)), float(m.group(4))
                roll, pitch, yaw = float(m.group(5)), float(m.group(6)), float(m.group(7))
                idx.positions_by_frame[curr_frame][rec_id] = (
                    carla.Location(x=lx, y=ly, z=lz),
                    carla.Rotation(pitch=pitch, yaw=yaw, roll=roll),
                )
                continue

            m = dyn_re.match(s)
            if m and curr_frame is not None:
                rec_id = int(m.group(1))
                lvx, lvy, lvz = float(m.group(2)), float(m.group(3)), float(m.group(4))
                avx, avy, avz = float(m.group(5)), float(m.group(6)), float(m.group(7))
                idx.velocities_by_frame[curr_frame][rec_id] = (
                    carla.Vector3D(x=lvx, y=lvy, z=lvz),
                    carla.Vector3D(x=avx, y=avy, z=avz),
                )
                continue

        # Build frame_times[0..frames-1]
        n = max(idx.frames, (max(times_tmp) if times_tmp else 0))
        idx.frame_times = [0.0] * n
        for k, v in times_tmp.items():
            if 1 <= k <= n:
                idx.frame_times[k - 1] = v
        return idx


class IdMapper:
    """
    Recorder-ID → runtime Actor.id via (type_id, optional role_name, nearest creation location).
    Includes a one-time bootstrap for static traffic.* actors.
    """
    def __init__(self, rec_idx: RecorderIndex, debug: bool = False):
        self.rec_idx = rec_idx
        self.debug = debug
        self.rec_to_run: Dict[int, int] = {}
        self.run_to_rec: Dict[int, int] = {}
        self._bootstrapped = False

    def _eligible(self, a: carla.Actor) -> bool:
        tid = (a.type_id or "").lower()
        return not (tid.startswith("sensor.") or tid == "spectator")

    @staticmethod
    def _dist(a: carla.Actor, loc: carla.Location) -> float:
        al = a.get_location()
        dx, dy, dz = al.x - loc.x, al.y - loc.y, al.z - loc.z
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    @staticmethod
    def _tol(tid: str) -> float:
        t = tid.lower()
        if t.startswith(("vehicle.", "walker.")):
            return 30.0
        if t.startswith("traffic."):
            return 15.0
        return 25.0

    def bootstrap_statics_once(self, world: carla.World):
        if self._bootstrapped:
            return
        actors = [a for a in world.get_actors() if self._eligible(a)]
        traffic_rt = [a for a in actors if (a.type_id or "").lower().startswith("traffic.")]
        for rec_id, (t, loc, rn, _) in self.rec_idx.creates_by_recid.items():
            if rec_id in self.rec_to_run or not t.lower().startswith("traffic."):
                continue
            cands = [a for a in traffic_rt if (a.type_id or "").lower() == t.lower() and a.id not in self.run_to_rec]
            if not cands:
                continue
            best = min(cands, key=lambda a: self._dist(a, loc))
            self.rec_to_run[rec_id] = best.id
            self.run_to_rec[best.id] = rec_id
        self._bootstrapped = True

    def get_runtime_id(self, world: carla.World, rec_id: int) -> Optional[int]:
        if rec_id in self.rec_to_run:
            return self.rec_to_run[rec_id]
        rec = self.rec_idx.creates_by_recid.get(rec_id)
        if not rec:
            return None
        t, loc, rn, _ = rec
        actors = [a for a in world.get_actors() if self._eligible(a)]
        actors = [a for a in actors if a.id not in self.run_to_rec and (a.type_id or "").lower() == t.lower()]
        if rn:
            rn_l = rn.strip().lower()
            by_rn = [a for a in actors if a.attributes.get("role_name", "").strip().lower() == rn_l]
            if by_rn:
                actors = by_rn
        if not actors:
            return None
        if len(actors) == 1:
            best = actors[0]
        else:
            best = min(actors, key=lambda a: self._dist(a, loc))
            if self._dist(best, loc) > self._tol(t) and not t.lower().startswith("traffic."):
                return None
        self.rec_to_run[rec_id] = best.id
        self.run_to_rec[best.id] = rec_id
        return best.id


def collisions_for_time_window(
    rec_idx: RecorderIndex,
    mapper: IdMapper,
    world: carla.World,
    sim_time_rel: float,
    half_window: float
) -> Dict[int, List[int]]:
    """
    Return {runtime_actor_id: [other_runtime_actor_ids]} for ALL recorder frames whose
    timestamps fall within [sim_time_rel - half_window, sim_time_rel + half_window].

    - WORLD collisions (4294967295) are ignored here because they have no runtime actor id.
      If you want to surface them, handle separately in the caller (e.g., add -1).
    - Duplicate pairs inside the window are de-duplicated.
    """
    ft = rec_idx.frame_times
    if not ft:
        return {}

    lo = sim_time_rel - half_window
    hi = sim_time_rel + half_window

    i0 = max(0, bisect.bisect_left(ft, lo) - 1)  # expand one left as safety
    i1 = min(len(ft), bisect.bisect_right(ft, hi) + 1)

    out_sets: Dict[int, Set[int]] = defaultdict(set)

    for k in range(i0, i1):
        t = ft[k]
        if not (lo <= t <= hi):
            continue
        frame_idx = k + 1  # recorder frames are 1-based
        pairs = rec_idx.collisions_by_frame.get(frame_idx, [])
        if not pairs:
            continue

        for rec_a, rec_b in pairs:
            # Skip WORLD collisions here (no runtime id to attach)
            if rec_a == WORLD_REC_ID or rec_b == WORLD_REC_ID:
                continue

            run_a = mapper.get_runtime_id(world, rec_a) if rec_idx.creates_by_recid.get(rec_a) else None
            run_b = mapper.get_runtime_id(world, rec_b) if rec_idx.creates_by_recid.get(rec_b) else None

            if run_a is None or run_b is None or run_a == run_b:
                continue

            out_sets[run_a].add(run_b)
            out_sets[run_b].add(run_a)

    # Convert to sorted lists for stable output
    return {aid: sorted(list(others)) for aid, others in out_sets.items()}
