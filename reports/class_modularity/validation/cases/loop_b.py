from __future__ import annotations
from .loop_a import A


class B:
    def __init__(self, peer: A):
        self.peer = peer
