class PathRegister:
    __instance = None
    paths = []

    def __new__(cls):
        if cls.__instance is None:
            cls.__instance = super().__new__(cls)
        return cls.__instance

    @classmethod
    def add_path(cls, path):
        cls.paths.append(path)

    @classmethod
    def get_paths(cls):
        return cls.paths


if __name__ == '__main__':
    PathRegister.add_path('a')

    print(PathRegister.get_paths())