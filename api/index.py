"""Vercel adapter for the same bounded API used by the local editor."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from climbsim.editor_server import make_handler  # noqa: E402

# Static assets are served by Vercel's CDN, not this function. The handler
# only receives /api/* rewrites and uses a terminating per-request worker.
class handler(make_handler(ROOT / "docs" / "demo")):
    """Explicit class declaration lets Vercel's Python AST detector find us."""
