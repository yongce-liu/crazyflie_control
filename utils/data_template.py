import numpy as np

# Note that all data is stored by the type of npz


class DataLoader():
    def __init__(self, path) -> None:
        assert path.split('.')[-1] in ['npz'], 'Data format is not supported'
        data = np.load(path)
        self.labels = data.files
        for _label in self.labels:
            setattr(self, _label, data[_label])

    def __len__(self):
        return len(self.labels)
    
