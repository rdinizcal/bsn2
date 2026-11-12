"""Small inputs to characterize external tools; not part of the BSN scope."""
from external_library import ExternalBase


class SharedState:
    def __init__(self):
        self.x = 0

    def first(self):
        self.x += 1

    def second(self):
        self.x += 2


class SplitState:
    def __init__(self):
        self.x = self.y = 0

    def first(self):
        self.x += 1

    def second(self):
        self.y += 2


class CallBridge:
    def first(self):
        self.x += 1

    def second(self):
        self.first()


class OnlyConstructor:
    def __init__(self):
        self.x = 0


class Stateless:
    def first(self, x):
        return x + 1

    def second(self, x):
        return x + 2


class SeparateGetters:
    def __init__(self):
        self.x = self.y = 0

    def first(self):
        return self.x

    def second(self):
        return self.y


class InheritedLogging(ExternalBase):
    def first(self):
        self.x += 1
        self.get_logger().info('first')

    def second(self):
        self.y += 1
        self.get_logger().info('second')
