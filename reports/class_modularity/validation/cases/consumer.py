from .dependency import Dependency


class ConstructedConsumer:
    def __init__(self):
        self.dependency = Dependency()


class TypedConsumer:
    def __init__(self, dependency: Dependency):
        self.dependency = dependency


class UntypedConsumer:
    def __init__(self, dependency):
        self.dependency = dependency


class LocalDependency:
    pass


class LocalConsumer:
    def __init__(self):
        self.dependency = LocalDependency()
