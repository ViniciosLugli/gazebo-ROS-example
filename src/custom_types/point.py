from modules.subscribers import EulerData

class Point:
    POINT_TOLERANCE = 0.1

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def to_tuple(self):
        return (self.x, self.y)

    def __str__(self):
        return f"Point({self.x}, {self.y})"

    def __repr__(self):
        return f"Point({self.x}, {self.y})"

    def __eq__(self, _point):
        return abs(self.x - _point.x) < self.POINT_TOLERANCE and abs(self.y - _point.y) < self.POINT_TOLERANCE

    @staticmethod
    def from_euler_data(euler_data: EulerData):
        return Point(euler_data.x, euler_data.y)