
class PathData:
    def __init__(self):
        self.fresh = True

    def update(self, data):
        self.fresh = True

    def is_fresh(self):
        return self.fresh

    def set_grid(self, message_data):
        return
