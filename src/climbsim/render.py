"""Renders a route attempt to an MP4 using MuJoCo's offscreen renderer."""

import mujoco
import numpy as np


def make_renderer(width=960, height=720):
    """Returns a callable model-agnostic renderer factory. MuJoCo's Renderer is
    bound to one MjModel, so `feasibility.attempt_route` builds its own
    ClimberModel internally -- we pass a *factory* through instead of a bound
    Renderer, and it builds the real one on first use against that model."""
    return _RendererFactory(width, height)


class _RendererFactory:
    def __init__(self, width, height):
        self.width, self.height = width, height
        self._renderer = None
        self._model = None

    def bind(self, model):
        if self._model is not model:
            self._renderer = mujoco.Renderer(model, height=self.height, width=self.width)
            self._model = model
        return self._renderer


def write_mp4(frames, path, fps=30):
    import imageio
    frames = [f for f in frames if f is not None]
    if not frames:
        raise ValueError("no frames to write")
    imageio.mimwrite(path, frames, fps=fps, quality=7, macro_block_size=1)
