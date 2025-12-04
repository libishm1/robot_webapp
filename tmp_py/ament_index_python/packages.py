import os


def get_package_share_directory(package_name: str) -> str:
    """
    Minimal helper to resolve package directory for xacro.

    Prefers UR_DESCRIPTION_PATH, otherwise falls back to first entry
    in ROS_PACKAGE_PATH or a matching child directory.
    """
    override = os.environ.get("UR_DESCRIPTION_PATH")
    if override and os.path.isdir(override):
        return override

    search = os.environ.get("ROS_PACKAGE_PATH", "")
    for entry in search.split(os.pathsep):
        if not entry:
            continue
        candidate = os.path.join(entry, package_name)
        if os.path.isdir(candidate):
            return candidate
        if os.path.isdir(entry):
            # Best-effort fallback to the provided entry.
            return entry

    raise EnvironmentError(f"Package path for {package_name} not found")
