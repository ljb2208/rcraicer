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
        
        # self.x_values = np.array(data.iloc[:,3:9])                
        # self.y_values = np.array(data.iloc[:,12:])
        self.x_values = np.array(data.loc[:,["roll","u_x","u_y","yaw_mder","steer","throttle"]])
        self.y_values = np.array(data.loc[:,["act_roll_mder", "act_u_x","act_u_y", "act_yaw_mder"]])

    