"""Custom movable 3D objects for the MetaUrban × SANDO demo.

Two factories, both returning a TrafficObject subclass you spawn with engine.spawn_object(...) and move
each step with obj.set_position([x,y,z]) / obj.set_heading_theta(theta) (kinematic; collision is computed
by the SANDO loop, not Bullet):

  make_glb_class(name, glb_path, w, l, h, hpr_fix=(H,P,R), extra_scale=1.0)
      Loads a .glb, applies a per-asset orientation fix hpr_fix (degrees) so it stands the right way and
      faces +Y (== heading 0 in MetaUrban's object frame), then auto-scales so its longest extent == `l` m.
      (glTF/Quaternius assets vary: some are Z-up, some Y-up, some lie with length along Z — hence hpr_fix.)

  make_drone_class(name, span=1.1, ...)
      Builds a recognisable quadcopter from Panda3D's bundled box/sphere primitives (body + X-frame arms +
      4 rotor discs + a red nose at +Y). No external asset; orientation is exact (nose = +Y = heading).
"""
import os
import panda3d
from panda3d.core import LVecBase3
from metaurban.component.static_object.traffic_object import TrafficObject
from metaurban.constants import MetaUrbanType

_PMODELS = os.path.join(os.path.dirname(panda3d.__file__), "models")
_BOX = os.path.join(_PMODELS, "box.egg.pz")
_SPH = os.path.join(_PMODELS, "misc", "sphere.egg.pz")


def make_glb_class(class_name, glb_path, width=0.5, length=0.5, height=0.5,
                   hpr_fix=(0.0, 0.0, 0.0), extra_scale=1.0, ground=True):
    """TrafficObject subclass rendering `glb_path`, oriented by hpr_fix (deg) and scaled so longest extent==`length`.
    ground=True drops the mesh so its lowest point sits at the object origin z=0 (feet on the floor, no hover)."""

    class _GLB(TrafficObject):
        CLASS_NAME = MetaUrbanType.TRAFFIC_OBJECT
        MASS = 1

        def __init__(self, position, heading_theta, lane=None, random_seed=None, name=None):
            super(_GLB, self).__init__(position, heading_theta, lane, random_seed, name=name)
            self._w, self._l, self._h = width, length, height
            if self.render:
                model = self.loader.loadModel(glb_path)
                model.setHpr(float(hpr_fix[0]), float(hpr_fix[1]), float(hpr_fix[2]))
                try:
                    lo, hi = model.getTightBounds()
                    ext = max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]) or 1.0
                    model.setScale((length / ext) * extra_scale)
                except Exception:
                    model.setScale(extra_scale)
                model.reparentTo(self.origin)
                if ground:                       # sit feet on the floor (origin z=0), not hovering
                    try:
                        lo, hi = model.getTightBounds(self.origin)
                        model.setZ(model.getZ() - lo[2])
                    except Exception:
                        pass
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


def make_drone_class(class_name, span=1.1, body=0.34, rotor_r=0.24, height=0.28,
                     body_col=(0.13, 0.14, 0.18, 1.0), rotor_col=(0.04, 0.04, 0.06, 1.0),
                     nose_col=(1.0, 0.25, 0.12, 1.0)):
    """TrafficObject subclass that is a procedural quadcopter (nose = +Y = heading), ~`span` m across."""

    class _Drone(TrafficObject):
        CLASS_NAME = MetaUrbanType.TRAFFIC_OBJECT
        MASS = 1

        def __init__(self, position, heading_theta, lane=None, random_seed=None, name=None):
            super(_Drone, self).__init__(position, heading_theta, lane, random_seed, name=name)
            self._w, self._l, self._h = span, span, height
            if not self.render:
                return
            root = self.origin.attachNewNode("drone")

            def _prim(path, sx, sy, sz, x, y, z, col):
                m = self.loader.loadModel(path)
                lo, hi = m.getTightBounds()
                ctr = (lo + hi) * 0.5
                ext = LVecBase3(max(hi[0] - lo[0], 1e-6), max(hi[1] - lo[1], 1e-6), max(hi[2] - lo[2], 1e-6))
                m.setPos(-ctr[0], -ctr[1], -ctr[2])          # recenter to local origin
                n = root.attachNewNode("p")
                m.reparentTo(n)
                n.setScale(sx / ext[0], sy / ext[1], sz / ext[2])
                n.setPos(x, y, z)
                n.setColor(*col)
                return n

            a = span * 0.5 * 0.70                            # rotor offset along each diagonal
            arm_len = (a * 2.0) * (2 ** 0.5)                 # bar spans corner-to-corner
            # two crossed arm bars (X-frame)
            for h in (45.0, -45.0):
                bar = _prim(_BOX, arm_len, 0.06, 0.05, 0, 0, 0, body_col)
                bar.setH(h)
            _prim(_BOX, body * 1.15, body, height * 0.55, 0, 0, 0, body_col)   # central body
            for sx in (-a, a):                               # 4 rotor discs at the arm ends
                for sy in (-a, a):
                    _prim(_SPH, rotor_r * 2, rotor_r * 2, 0.05, sx, sy, height * 0.32, rotor_col)
            _prim(_BOX, body * 0.55, 0.10, 0.08, body * 0.78, 0, 0.02, nose_col)  # red nose -> +X forward (heading 0)
            self._model = root
            try:
                import os as _os
                from panda3d.core import BitMask32
                if _os.environ.get("DRONE_COLLIDE", "0") != "1":   # debug: reproduce the glitch
                    self.body.setIntoCollideMask(BitMask32.allOff())   # visual marker ONLY: a kinematic
                #   teleporting body that still collides launches grazed dynamic agents (the flying-
                #   bicycle glitch, 2026-07-08); clearance accounting is ours, not Bullet's
            except Exception:
                pass

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

    _Drone.__name__ = class_name
    return _Drone
