import re
from collections import defaultdict
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple

import carla

from carla_data_classes.dynamic.DataCollision import DataCollision
from carla_data_classes.enums.DataCollisionKind import DataCollisionKind


# ---------- Recorder parsing & mapping ----------------------------------------

@dataclass
class _CreateRecord:
    """One 'Create' line from the recorder info."""
    rec_id: int  # recorder ID
    type_id: str  # e.g., 'vehicle.mini.cooper_s_2021'
    role_name: Optional[str]  # e.g., 'hero'
    loc: carla.Location  # creation location from info text
    created_frame: int  # frame the Create* appeared in


@dataclass
class _RecordingIndex:
    delta_seconds: float
    frames: int
    creates_by_recid: Dict[int, _CreateRecord]
    collisions_by_frame: Dict[int, List[Tuple[int, int]]]  # [(rec_id1, rec_id2), ...]


def _parse_recorder_info(info: str) -> _RecordingIndex:
    """
    Parse the full string returned by client.show_recorder_file_info(path, True).
    We collect:
      - delta seconds per frame
      - total frame count
      - 'Create ...' actor descriptors keyed by recorder ID
      - per-frame collisions: list of (rec_id_a, rec_id_b)
    """
    # frame headers
    frame_hdr = re.compile(r"^Frame\s+(\d+)\s+at\s+([0-9.]+)\s+seconds$")
    # Create lines
    # Example: " Create 24: vehicle.mini.cooper_s_2021 (1) at (-5534.77, 14534.3, 8.70507)"
    create_re = re.compile(
        r"^\s*Create\s+\d+:\s+([A-Za-z0-9_.]+)\s+\(\d+\)\s+at\s+\((-?[0-9.]+),\s*(-?[0-9.]+),\s*(-?[0-9.]+)\)\s*$"
    )
    # Collisions inside a frame:
    #   " Collision id 580 between 42 (hero)  with 15"
    coll_re = re.compile(r"^\s*Collision\s+id\s+\d+\s+between\s+(\d+).*\s+with\s+(\d+)\s*$")

    lines = info.splitlines()
    curr_frame = None
    delta_seconds = None
    frames_seen = set()

    creates_by_recid: Dict[int, _CreateRecord] = {}
    collisions_by_frame: Dict[int, List[Tuple[int, int]]] = defaultdict(list)

    # Support quick lookup of recorder ID that the following attribute lines belong to
    last_create_rec_id: Optional[int] = None

    for idx, line in enumerate(lines):
        # Frame header
        m = frame_hdr.match(line.strip())
        if m:
            curr_frame = int(m.group(1))
            frames_seen.add(curr_frame)

            # "Frame 2 at X seconds" gives us dt
            if curr_frame == 2 and delta_seconds is None:
                try:
                    delta_seconds = float(m.group(2))
                except Exception:
                    delta_seconds = None
            continue

        # Create lines
        m = create_re.match(line)
        if m and curr_frame is not None:
            type_id = m.group(1)
            # The recorder ID is NOT the "(N)" in the Create line; it's printed on a separate line later for positions,
            # but the very next indented lines include attributes and (crucially) "role_name = hero".
            # To get the recorder ID, we read ahead until we find "role_name =" or the "Create X:" id from the sample.
            # In CARLA recorder info, the recorder ID is the number used elsewhere (e.g., "Id: 24 ...").
            # We can recover it reliably from the "Create XX:" number: it's printed at the start of this line before ":".
            # So we extract it directly via another regex on the same line.
            # Example: " Create 24: vehicle...."
            m2 = re.match(r"^\s*Create\s+(\d+):", line)
            rec_id = int(m2.group(1)) if m2 else -1

            x, y, z = float(m.group(2)), float(m.group(3)), float(m.group(4))
            creates_by_recid[rec_id] = _CreateRecord(
                rec_id=rec_id,
                type_id=type_id,
                role_name=None,
                loc=carla.Location(x=x, y=y, z=z),
                created_frame=curr_frame,
            )
            last_create_rec_id = rec_id
            continue

        # Read attributes after Create (indented), we only care about role_name for robust mapping
        if last_create_rec_id is not None and line.startswith("  "):
            if "role_name" in line:
                # "  role_name = hero"
                try:
                    rn = line.split("=", 1)[1].strip()
                    creates_by_recid[last_create_rec_id].role_name = rn
                except Exception:
                    pass
            # continue scanning attribute block
            continue
        else:
            last_create_rec_id = None

        # Collision lines within a frame
        m = coll_re.match(line)
        if m and curr_frame is not None:
            a, b = int(m.group(1)), int(m.group(2))
            collisions_by_frame[curr_frame].append((a, b))
            continue

    # Fallbacks
    if delta_seconds is None:
        # Last resort: estimate from last frame time if present, else assume 0.05
        m = re.search(r"^Frame\s+(\d+)\s+at\s+([0-9.]+)\s+seconds$", info, re.MULTILINE)
        delta_seconds = 0.05

    # Try explicit "Frames: N" at bottom
    m_frames = re.search(r"^Frames:\s+(\d+)", info, re.MULTILINE)
    total_frames = int(m_frames.group(1)) if m_frames else (max(frames_seen) if frames_seen else 0)

    return _RecordingIndex(
        delta_seconds=delta_seconds,
        frames=total_frames,
        creates_by_recid=creates_by_recid,
        collisions_by_frame=collisions_by_frame,
    )


WORLD_REC_ID = 4294967295


def _kind_from_type_id(type_id: Optional[str]) -> DataCollisionKind:
    if not type_id:
        return DataCollisionKind.OTHER
    t = type_id.lower()
    if t.startswith("vehicle."):
        return DataCollisionKind.VEHICLE
    if t.startswith("walker."):
        return DataCollisionKind.WALKER
    if t.startswith("traffic."):
        return DataCollisionKind.TRAFFIC
    if t == "world":
        return DataCollisionKind.STATIC
    return DataCollisionKind.OTHER


class _IdMapper:
    def __init__(self, rec_idx: _RecordingIndex, debug: bool = False):
        self.rec_idx = rec_idx
        self.rec_to_run: Dict[int, int] = {}
        self.run_to_rec: Dict[int, int] = {}
        self.debug = debug
        # NEW: keep a flag so we only pre-map static stuff once
        self._static_bootstrapped = False

    def get_runtime_id(self, world: carla.World, rec_id: int, *, actors_cache=None) -> Optional[int]:
        """Public API used by collisions.py. Returns the runtime Actor.id for a recorder ID, or None."""
        # make sure statics are bootstrapped before first lookup
        try:
            self._bootstrap_static_once(world)
        except AttributeError:
            # if you don't use the bootstrap version, ignore
            pass

        if rec_id in self.rec_to_run:
            return self.rec_to_run[rec_id]

        return self._match_one(world, rec_id, actors_cache=actors_cache)

    def get_recorder_id(self, runtime_actor_id: int) -> Optional[int]:
        return self.run_to_rec.get(runtime_actor_id)

    def _eligible(self, a: carla.Actor) -> bool:
        tid = (a.type_id or "").lower()
        return not (tid.startswith("sensor.") or tid == "spectator")

    def _distance(self, a: carla.Actor, loc: carla.Location) -> float:
        al = a.get_location()
        dx, dy, dz = (al.x - loc.x), (al.y - loc.y), (al.z - loc.z)
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def _tol_for(self, type_id: str) -> float:
        t = type_id.lower()
        if t.startswith(("vehicle.", "walker.")):
            return 30.0  # was tighter; vehicles can be far from their creation pose when we map
        if t.startswith("traffic."):
            return 15.0  # ↑ widen for traffic lights/signs
        return 25.0

    def _match_one(self, world: carla.World, rec_id: int, actors_cache=None) -> Optional[int]:
        if rec_id in self.rec_to_run:
            return self.rec_to_run[rec_id]

        rec = self.rec_idx.creates_by_recid.get(rec_id)
        if rec is None:
            if self.debug:
                print(f"[mapper] no Create record for rec_id={rec_id}")
            return None

        actors = list(actors_cache) if actors_cache is not None else list(world.get_actors())
        actors = [a for a in actors if self._eligible(a)]
        actors = [a for a in actors if a.id not in self.run_to_rec]

        # filter by type_id
        typed = [a for a in actors if (a.type_id or "").lower() == rec.type_id.lower()]
        if not typed:
            if self.debug:
                print(f"[mapper] rec_id {rec_id}: no runtime actors for type {rec.type_id}")
            return None

        # prefer role_name match
        if rec.role_name:
            rn = rec.role_name.strip().lower()
            typed_rn = [a for a in typed if a.attributes.get("role_name", "").strip().lower() == rn]
            if typed_rn:
                typed = typed_rn

        # sole candidate → accept
        if len(typed) == 1:
            chosen = typed[0]
            self.rec_to_run[rec_id] = chosen.id
            self.run_to_rec[chosen.id] = rec_id
            return chosen.id

        # otherwise choose nearest to the recorder "Create" location
        tol = self._tol_for(rec.type_id)
        best, best_d = None, 1e9
        for a in typed:
            d = self._distance(a, rec.loc)
            if d < best_d:
                best, best_d = a, d

        if best is not None and best_d <= tol:
            self.rec_to_run[rec_id] = best.id
            self.run_to_rec[best.id] = rec_id
            return best.id

        # last resort for statics: if multiple candidates but all far, still bind the nearest traffic.*
        if rec.type_id.lower().startswith("traffic.") and best is not None:
            self.rec_to_run[rec_id] = best.id
            self.run_to_rec[best.id] = rec_id
            if self.debug:
                print(f"[mapper] rec_id {rec_id}: forced nearest traffic.* {best.id} at {best_d:.2f}m")
            return best.id

        if self.debug:
            print(f"[mapper] rec_id {rec_id}: nearest {best.id if best else None} at {best_d:.2f}m > tol {tol}")
        return None

    def _bootstrap_static_once(self, world: carla.World) -> None:
        """Map all traffic.* recorder IDs to nearest runtime traffic actors exactly once."""
        if self._static_bootstrapped:
            return
        actors = [a for a in world.get_actors() if self._eligible(a)]
        traffic_rt = [a for a in actors if (a.type_id or "").lower().startswith("traffic.")]
        if not traffic_rt:
            return
        for rec_id, rec in self.rec_idx.creates_by_recid.items():
            if rec_id in self.rec_to_run:
                continue
            if not rec.type_id.lower().startswith("traffic."):
                continue
            # pick nearest runtime traffic of same type_id
            candidates = [a for a in traffic_rt if
                          (a.type_id or "").lower() == rec.type_id.lower() and a.id not in self.run_to_rec]
            if not candidates:
                continue
            best = min(candidates, key=lambda a: self._distance(a, rec.loc))
            self.rec_to_run[rec_id] = best.id
            self.run_to_rec[best.id] = rec_id
        self._static_bootstrapped = True

    def update_until_frame(self, world: carla.World, frame_idx: int) -> None:
        actors_cache = world.get_actors()
        # first: static bootstrap (does nothing after first time)
        self._bootstrap_static_once(world)

        # then: normal per-create mapping up to current frame
        for rec_id, rec in self.rec_idx.creates_by_recid.items():
            if rec.created_frame <= frame_idx and rec_id not in self.rec_to_run:
                self._match_one(world, rec_id, actors_cache=actors_cache)


def _collisions_for_frame(frame_idx: int, world: carla.World, mapper: _IdMapper, rec_idx: _RecordingIndex) -> Dict[
    int, List[DataCollision]]:
    per_actor: Dict[int, List[DataCollision]] = defaultdict(list)
    events = rec_idx.collisions_by_frame.get(frame_idx, [])

    for rec_a, rec_b in events:
        # handle "world" on either side
        if rec_a == WORLD_REC_ID or rec_b == WORLD_REC_ID:
            rec_other = rec_b if rec_a == WORLD_REC_ID else rec_a
            run_other = mapper.get_runtime_id(world, rec_other)

            # figure out the recorder type_id of the mapped side (for kind); world side is "WORLD"
            rec_other_create = rec_idx.creates_by_recid.get(rec_other)
            type_other = rec_other_create.type_id if rec_other_create else "unknown"

            coll = DataCollision(
                actor1_kind=_kind_from_type_id(type_other),
                actor2_kind=DataCollisionKind.STATIC,
                actor1_id=(run_other if run_other is not None else -1),
                actor1_type_id=type_other,
                actor2_id=-1,
                actor2_type_id="WORLD",
            )
            if run_other is not None:
                per_actor[run_other].append(coll)
            # we cannot attach to 'world', so we’re done for this pair
            continue

        # normal actor↔actor mapping
        rec_a_create = rec_idx.creates_by_recid.get(rec_a)
        rec_b_create = rec_idx.creates_by_recid.get(rec_b)

        run_a = mapper.get_runtime_id(world, rec_a) if rec_a_create else None
        run_b = mapper.get_runtime_id(world, rec_b) if rec_b_create else None

        type_a = rec_a_create.type_id if rec_a_create else "unknown"
        type_b = rec_b_create.type_id if rec_b_create else "unknown"

        coll = DataCollision(
            actor1_kind=_kind_from_type_id(type_a),
            actor2_kind=_kind_from_type_id(type_b),
            actor1_id=(run_a if run_a is not None else -1),
            actor1_type_id=type_a,
            actor2_id=(run_b if run_b is not None else -1),
            actor2_type_id=type_b,
        )
        if run_a is not None:
            per_actor[run_a].append(coll)
        if run_b is not None:
            per_actor[run_b].append(coll)

    return per_actor
