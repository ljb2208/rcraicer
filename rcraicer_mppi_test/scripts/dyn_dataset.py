import torch
from torch.utils.data import Dataset
import pandas as pd
import numpy as np

from  sklearn import preprocessing

class DynDataset(Dataset):
    def __init__(self, filename, norm=False):
        self.x_values = None
        self.y_values = None
        self.load_data(filename, norm)

    def __len__(self):
        return len(self.x_values)
    
    def __getitem__(self, idx):
        return {'x_values' : self.x_values[idx], 'y_values' : self.y_values[idx]}

    def load_data(self, filename, norm):
        data = pd.read_csv(filename)


        if (norm == True):
            mean = data.mean()
            std = data.std()

            norm_data = data.values
            min_max_scalar = preprocessing.MinMaxScaler(feature_range=(-1,1))
            scaled = min_max_scalar.fit_transform(norm_data)            

            data = pd.DataFrame(scaled, columns=data.columns)
        
        # self.x_values = np.array(data.iloc[:,3:9])                
        # self.y_values = np.array(data.iloc[:,12:])
        #        

        # yaw = np.array(data.loc[:,["yaw"]])        
        # u_x = np.array(data.loc[:,["u_x"]])        
        # u_y = np.array(data.loc[:,["u_y"]])        
        # act_yaw = np.array(data.loc[:,["act_yaw"]])
        # act_u_x = np.array(data.loc[:,["act_u_x"]])        
        # act_u_y = np.array(data.loc[:,["act_u_y"]])                

        # data["comp_u_x"] = np.cos(yaw) * u_x + np.sin(yaw) * u_y
        # data["comp_u_y"] = np.sin(yaw) * u_x + np.cos(yaw) * u_y

        # data["act_comp_u_x"] = np.cos(act_yaw) * act_u_x + np.sin(act_yaw) * act_u_y
        # data["act_comp_u_y"] = np.sin(act_yaw) * act_u_x + np.cos(act_yaw) * act_u_y        

        self.x_values = np.array(data.loc[:,["roll","comp_u_x","comp_u_y","yaw_mder","steer","throttle"]])
        self.y_values = np.array(data.loc[:,["act_roll_mder","accel_x","accel_y","accel_yaw"]])

        # if (norm):
        #     normv = 
        #     self.x_values = (self.x_values - mean) / std
        #     self.y_values = (self.y_values - mean) / std

        # self.y_values = np.array(data.loc[:,["act_roll","accel_x","accel_y","act_yaw_mder"]])
        

        # self.y_values = np.array(data.loc[:,["act_roll", "act_comp_u_x","act_comp_u_y", "act_yaw_mder"]])

    