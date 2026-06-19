"""CustomGLBObject — spawn an arbitrary .glb mesh as a movable object in MetaUrban's 3D scene.

Mirrors the TrafficCone pattern (component/static_object/traffic_object.py): a TrafficObject subclass
that loads a glb onto self.origin. Auto-scales the mesh so its longest horizontal extent == `length`
(glb native units are not metres). Move it each step with obj.set_position([x,y,z]) — the visual
follows (we drive it kinematically; collision is computed by the SANDO loop, not Bullet).

Usage:
    from custom_glb_object import make_glb_class
    Drone = make_glb_class("Drone", "/abs/path/drone.glb", width=0.5, length=0.5, height=0.3, up_fix=True)
    drone = engine.spawn_object(Drone, position=[x, y], heading_theta=0.0)
    ...
    drone.set_position([x, y, z]); drone.set_heading_theta(theta)
"""
from panda3d.core import LVector3
from metaurban.component.static_object.traffic_object import TrafficObject
from metaurban.constants import MetaUrbanType


def make_glb_class(class_name, glb_path, width=0.5, length=0.5, height=0.5,
                   up_fix=True, extra_scale=1.0, z_lift=0.0):
    """Build a TrafficObject subclass that renders `glb_path`, auto-scaled to ~`length` metres."""

    class _GLB(TrafficObject):
        CLASS_NAME = MetaUrbanType.TRAFFIC_OBJECT   # must be a valid MetaUrbanType (body node type)
        MASS = 1

        def __init__(self, position, heading_theta, lane=None, random_seed=None, name=None):
            super(_GLB, self).__init__(position, heading_theta, lane, random_seed, name=name)
            self._w, self._l, self._h = width, length, height
            if self.render:
                model = self.loader.loadModel(glb_path)
                # glTF is Y-up; MetaUrban/Panda3D is Z-up -> rotate +90deg about X so the mesh stands up.
                if up_fix:
                    model.setP(90)
                # auto-scale: longest tight-bounds extent -> `length` metres
                try:
                    lo, hi = model.getTightBounds()
                    ext = max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]) or 1.0
                    s = (length / ext) * extra_scale
                except Exception:
                    s = extra_scale
                model.setScale(s)
                model.setZ(z_lift)
                model.reparentTo(self.origin)
                self._model = model

        @property
        def WIDTH(self): return self._w

        @property
        def LENGTH(self): return self._l

        @property
        def HEIGHT(self): return self._h

        @property
        def top_down_width(self): return self._w

        @property
        def top_down_length(self): return self._l

    _GLB.__name__ = class_name
    return _GLB
