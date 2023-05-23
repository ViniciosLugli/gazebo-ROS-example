from collections import deque
from custom_types import Point
import csv

class PathController(deque):
    def __init__(self, csv_path: str = None):
        super().__init__()

        if csv_path is not None:
            with open(csv_path, newline='') as csvfile:
                reader = csv.reader(csvfile, delimiter=',')
                for row in reader:
                    self.enqueue(Point(float(row[0]), float(row[1])))

    def enqueue(self, point: Point):
        super().append(point)

    def dequeue(self):
        return super().popleft()

    def get_next_point(self) -> Point:
        return self.dequeue()

    def get_current_point(self) -> Point:
        if len(self) == 0:
            return None
        return self[0]

    def is_point_reached(self, point: Point) -> bool:
        return point == self.get_current_point()
