"""Load the locally built HPIPM Python bindings on Windows.

The vendored `hpipm_python` wrapper hard-codes ``ctypes.CDLL('libhpipm.so')``,
which does not exist on Windows.  Rather than patching files outside the
repository, this module redirects that one ``CDLL`` call to the real DLL and puts
the BLASFEO directory on the DLL search path before the wrapper is imported.

Locations (override with environment variables):
    HPIPM_LIB      default  D:\\Tools\\hpipm\\build\\libhpipm.dll
    BLASFEO_LIB    default  D:\\Tools\\blasfeo\\build\\libblasfeo.dll
    HPIPM_PYTHON   default  D:\\Tools\\hpipm\\interfaces\\python\\hpipm_python\\build\\lib

`available()` reports whether the bindings can be used, so callers can fall back
to IPOPT when HPIPM is not installed.
"""
from __future__ import annotations

import ctypes
import os
import sys
from pathlib import Path

DEFAULTS = {
    "HPIPM_LIB": r"D:\Tools\hpipm\build\libhpipm.dll",
    "BLASFEO_LIB": r"D:\Tools\blasfeo\build\libblasfeo.dll",
    "HPIPM_PYTHON": r"D:\Tools\hpipm\interfaces\python\hpipm_python\build\lib",
}

_STATE = {"loaded": None, "error": ""}


def _paths():
    out = {}
    for key, default in DEFAULTS.items():
        v = os.environ.get(key) or default
        out[key] = Path(v)
    return out


def _prepare():
    p = _paths()
    for key in ("HPIPM_LIB", "BLASFEO_LIB"):
        d = p[key].parent
        if d.is_dir():
            os.environ["PATH"] = str(d) + os.pathsep + os.environ.get("PATH", "")
            try:
                os.add_dll_directory(str(d))
            except (AttributeError, OSError):
                pass
    if str(p["HPIPM_PYTHON"]) not in sys.path:
        sys.path.insert(0, str(p["HPIPM_PYTHON"]))

    dll = str(p["HPIPM_LIB"])
    blasfeo_dll = str(p["BLASFEO_LIB"])
    orig = ctypes.CDLL

    def patched(name, *a, **k):
        n = str(name)
        if n in ("libhpipm.so", "libhpipm.dylib"):
            n = dll
        elif n in ("libblasfeo.so", "libblasfeo.dylib"):
            n = blasfeo_dll
        return orig(n, *a, **k)

    ctypes.CDLL = patched


def load():
    """(ok, module_dict_or_error_message).  Cached."""
    if _STATE["loaded"] is not None:
        return _STATE["loaded"], _STATE["error"]
    p = _paths()
    if not p["HPIPM_LIB"].exists():
        _STATE["loaded"] = False
        _STATE["error"] = "libhpipm.dll not found at %s" % p["HPIPM_LIB"]
        return False, _STATE["error"]
    if p["BLASFEO_LIB"].exists() is False:
        _STATE["error"] = "warning: libblasfeo.dll not found at %s" % p["BLASFEO_LIB"]
    _prepare()
    try:
        from hpipm_python import (hpipm_ocp_qp, hpipm_ocp_qp_dim,   # noqa: F401
                                  hpipm_ocp_qp_sol, hpipm_ocp_qp_solver,
                                  hpipm_ocp_qp_solver_arg)
        _STATE["loaded"] = True
        return True, ""
    except Exception as exc:                                  # noqa: BLE001
        _STATE["loaded"] = False
        _STATE["error"] = "%s: %s" % (type(exc).__name__, exc)
        return False, _STATE["error"]


def available():
    return load()[0]


def classes():
    """The five wrapper classes, importing on demand.  Raises if unavailable."""
    ok, err = load()
    if not ok:
        raise RuntimeError("HPIPM python bindings unavailable: %s" % err)
    from hpipm_python import (hpipm_ocp_qp, hpipm_ocp_qp_dim, hpipm_ocp_qp_sol,
                              hpipm_ocp_qp_solver, hpipm_ocp_qp_solver_arg)
    return dict(qp=hpipm_ocp_qp, dim=hpipm_ocp_qp_dim, sol=hpipm_ocp_qp_sol,
                solver=hpipm_ocp_qp_solver, arg=hpipm_ocp_qp_solver_arg)


if __name__ == "__main__":
    ok, err = load()
    print("HPIPM available:", ok, err)
    if ok:
        print("paths:", {k: str(v) for k, v in _paths().items()})
