# carla_data_classes/__init__.py

def ensure_core_types(ns: dict) -> None:
    """
    Injects only the core types used in base-class annotations into the caller's
    module globals. Safe to call from any dynamic actor module (no circular imports).
    """
    from .dynamic.DataBoundingBox import DataBoundingBox
    from .dynamic.DataCollision import DataCollision
    from .static.DataLocation import DataLocation
    from .static.DataRotation import DataRotation
    from typing import Optional
    # optional vector type; ignore if not present
    try:
        from .static.DataVector3D import DataVector3D  # noqa: F401
    except Exception:
        DataVector3D = None  # type: ignore

    for name, val in [
        ("DataBoundingBox", DataBoundingBox),
        ("DataCollision", DataCollision),
        ("DataLocation", DataLocation),
        ("DataRotation", DataRotation),
        ("DataVector3D", DataVector3D),
        ("Optional", Optional),
    ]:
        if val is not None and name not in ns:
            ns[name] = val
