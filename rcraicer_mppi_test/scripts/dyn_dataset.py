import torch
from torch.utils.data import Dataset
import pandas as pd
import numpy as np

class DynDataset(Dataset):
    def __init__(self, filename):
        self.x_values = None
        self.y_values = None
        self.load_data(filename)

    def __len__(self):
        return len(self.x_values)
    
    def __getitem__(self, idx):
        return {'x_values' : self.x_values[idx], 'y_values' : self.y_values[idx]}

    def load_data(self, filename):
        data = pd.read_csv(filename)
        
        self.x_values = np.array(data.iloc[:,0:9])                
        self.y_values = np.array(data.iloc[:,13:])

    