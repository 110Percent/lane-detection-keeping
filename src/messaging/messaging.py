import datetime


class Message:
    def __init__(self, data, sender, receiver):
        self.lanes = self.get_lanes(data)
        self.h_samples = self.get_h_samples(data)
        self.raw_file = self.get_raw_file(data)
        self.sender = sender
        self.receiver = receiver
        self.timestamp = datetime.now()

    def get_lanes(self, data):
        return data['lanes']

    def get_h_samples(self, data):
        return data['h_samples']
    
    def get_raw_file(self, data):
        return data['img_path']

    def send(self):
        return self.message