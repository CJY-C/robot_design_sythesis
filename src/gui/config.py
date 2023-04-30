# config.py
import json

class Config:
    def __init__(self):
        self.task_space = {"width": 1, "height": 1, "length": 1}
        self.obstacle = {"width": 1, "height": 1, "length": 1}
        self.obstacle_matrix = [[[0 for _ in range(3)] for _ in range(3)] for _ in range(3)]  # 3D matrix
        self.target_points = []

    def add_target_point(self):
        self.target_points.append({"position": [0, 0, 0], "orientation": [0, 0, 0]})

    def remove_target_point(self, index):
        if 0 <= index < len(self.target_points):
            del self.target_points[index]

    def save_to_file(self, file_path="config.json"):
        with open(file_path, "w") as f:
            json.dump(self.__dict__, f, indent=2)
