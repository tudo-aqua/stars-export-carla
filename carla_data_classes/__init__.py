def ensure_runtime_types(ns: dict) -> None:
    # Import inside the function to avoid import cycles at import time.
    from .dynamic.DataBoundingBox import DataBoundingBox
    from .static.DataLocation import DataLocation
    from .static.DataRotation import DataRotation

    # Only set if not already present in the module's globals
    ns.setdefault("DataBoundingBox", DataBoundingBox)
    ns.setdefault("DataLocation", DataLocation)
    ns.setdefault("DataRotation", DataRotation)
