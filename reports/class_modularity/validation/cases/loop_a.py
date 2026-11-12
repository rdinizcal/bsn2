from __future__ import annotations
from .loop_b import B


class A:
    def __init__(self, peer: B):
        self.peer = peer
