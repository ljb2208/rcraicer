import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
from torch.utils.data.dataset import random_split
from dyn_dataset import DynDataset
from dyn_model import DynModel
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


class Dyn():
    def __init__(self, data_path, model_path, log_path, lr=0.0005, epochs=100, optim="rms", interactive=True):
        self.bs = 64      
        
        #self.lr = 0.0003  # good results with lr 0.0005 and epochs 1000 and SGDProp
        self.lr = lr
        self.epochs = epochs
        self.optim = optim
        self.interactive = interactive
        self.model_path = model_path
        self.data_path = data_path
        self.log_path = log_path
        self.run_id = 0

        self.log_file = open(self.log_path + "dynmodel.log","a")
        self.log_file_verbose =  open(self.log_path + "dynmodel_verbose.log","a")

    
        if torch.cuda.is_available():
            self.device = torch.device('cuda')
        else:
            self.device = torch.device('cpu')           
    

    def log(self, text):
        logtext = "{}\t{}".format(self.run_id, text)
        self.log_file.write(logtext)

    def log_verbose(self, text):
        logtext = "{}\t{}".format(self.run_id, text)
        self.log_file_verbose.write(logtext)

    def comp_values(self, x_values, y_values):
        print ("Comp:")

        for i in range(0, x_values.shape[1]):
            x_val = x_values[0][i]
            y_val = y_values[0][i]

            print("{}: {}/{} Diff: {}".format(i, x_val, y_val, abs(x_val - y_val)))

    
    def test(self):
        model = DynModel()
        model.load_state_dict(torch.load(self.model_path + "dynmodel_states{}.pth".format(self.run_id)))

        model.eval()

        test_filename = self.data_path + "dynamics_data_test.csv"

        test_ds = DynDataset(test_filename)        

        self.log_verbose("Testing samples: {}\n".format(len(test_ds)))

        lossfn = nn.SmoothL1Loss(reduction="sum")

        results = np.empty((test_ds.y_values.shape[0], 7))
        count = 0

        for test_rec in test_ds:
            x_data = torch.tensor([test_rec["x_values"]]).to(dtype=torch.float32)
            y_data = torch.tensor([test_rec["y_values"]]).to(dtype=torch.float32)
            output = model.forward(x_data)

            loss = lossfn(output, y_data)

            results[count][0] = output[0][0]
            results[count][1] = output[0][1]
            results[count][2] = output[0][2]
            results[count][3] = y_data[0][0]
            results[count][4] = y_data[0][1]
            results[count][5] = y_data[0][2]
            results[count][6] = loss
            
            # print("loss: {} \n".format(loss))
            # self.comp_values(output, y_data)
            count += 1        

        df = pd.DataFrame(data=results, dtype=results.dtype, columns=["u_x_out", "u_y_out", "yaw_mder_out", "u_x", "u_y", "yaw_mder", "loss"])

        loss_col = df["loss"]

        lmin = loss_col.min()
        lmax = loss_col.max()
        lavg = loss_col.mean()

        if self.interactive:
            print("Losses. Min: {} Max: {} Mean: {}\n".format(lmin, lmax, lavg))
            print("Top Losses_______________")

        self.log("Training Losses. Min: {} Max: {} Mean: {}\n".format(lmin, lmax, lavg))
        self.log_verbose("Top losses:\n")
        
        sorted_df = df.sort_values(by="loss", ascending=False)

        for i in range(0, 10):
            index = sorted_df.index[i]

            if self.interactive:
                print("Index: {} loss: {} u_x: {} u_y: {} yaw_mderr: {} u_x_out: {} u_y_out: {} yaw_out: {}".format(index, sorted_df.loss[index],sorted_df.u_x[index],
                                    sorted_df.u_y[index], sorted_df.yaw_mder[index], sorted_df.u_x_out[index], sorted_df.u_y_out[index], sorted_df.yaw_mder_out[index]))

                self.log_verbose("Index: {} loss: {} u_x: {} u_y: {} yaw_mderr: {} u_x_out: {} u_y_out: {} yaw_out: {}\n".format(index, sorted_df.loss[index],sorted_df.u_x[index],
                                    sorted_df.u_y[index], sorted_df.yaw_mder[index], sorted_df.u_x_out[index], sorted_df.u_y_out[index], sorted_df.yaw_mder_out[index]))

        fig, axarr = plt.subplots(2, 2, figsize=(12,8))
        loss_plot = df.plot(y="loss", kind="line", ax=axarr[0][0])
        x_out_plot = df.plot(y="u_x_out", kind="line", ax=axarr[0][1])
        x_out_plot = df.plot(y="u_x", kind="line", ax=axarr[0][1])
        y_out_plot = df.plot(y="u_y_out", kind="line", ax=axarr[1][0])
        y_out_plot = df.plot(y="u_y", kind="line", ax=axarr[1][0])

        yaw_out_plot = df.plot(y="yaw_mder_out", kind="line", ax=axarr[1][1])
        yaw_out_plot = df.plot(y="yaw_mder", kind="line", ax=axarr[1][1])

        plt.suptitle("id: {} lr: {} optim: {} epochs: {} losses min: {:.4f} max: {:.4f} mean: {:.4f}".format(self.run_id, self.lr, self.optim, self.epochs, lmin, lmax, lavg))
        plt.savefig(self.log_path + "graph{}.png".format(self.run_id))

        if self.interactive:
            plt.show()       
        
        plt.close()


        

    def write_settings(self):
        logtext = "lr: {} epochs: {} optim: {} bs: {}\n".format(self.lr, self.epochs, self.optim, self.bs)
        self.log(logtext)

    def learn(self):

        self.run_id += 1

        model = DynModel()
        model.to(self.device)

        self.write_settings()        

        training_filename = self.data_path + "dynamics_data.csv"
        # valid_filename = self.data_path + "dyn2_model_valid.csv"

        dynamics_ds = DynDataset(training_filename)

        ds_size = len(dynamics_ds)
        train_size = int(ds_size * 0.8)
        valid_size = ds_size - train_size

        train_ds, valid_ds = random_split(dynamics_ds, [train_size, valid_size])
        # valid_ds = DynDataset(valid_filename)

        train_data_loader = DataLoader(train_ds, batch_size=self.bs, shuffle=True, num_workers=0)
        valid_data_loader = DataLoader(valid_ds, batch_size=self.bs, shuffle=True, num_workers=0)

        num_training_samples  = len(train_ds)
        num_valid_samples = len(valid_ds)        

        self.log_verbose("Training samples: {} Validation samples: {}\n".format(num_training_samples, num_valid_samples))

        # lossfn = nn.MSELoss()

        optimizer = optim.RMSprop(model.parameters(), lr=self.lr, eps=1e-5, weight_decay=0.01)

        if self.optim == "adam":
            optimizer = optim.AdamW(model.parameters(), lr=self.lr, eps=1e-5, weight_decay=0.01)
         
        scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, self.epochs, eta_min=1e-5, verbose=False)
        # scheduler = optim.lr_scheduler.OneCycleLR(optimizer, max_lr=self.lr, steps_per_epoch=len(train_data_loader), epochs=self.epochs, verbose=True)

        train_loss = 0.
        valid_loss = 0.

        # lossfn = nn.MSELoss() #reduction="none")
        lossfn = nn.SmoothL1Loss(reduction="sum")

        for epoch in range(1, self.epochs + 1):
            model.train()

            for batch in enumerate(train_data_loader):
                x_values = batch[1]["x_values"].to(dtype=torch.float32).to(self.device)
                y_values = batch[1]["y_values"].to(dtype=torch.float32).to(self.device)

                optimizer.zero_grad()

                output = model.forward(x_values)
                # loss = lossfn.forward(output, y_values)  
                # flatten loss              
                loss = lossfn(output.view(-1), y_values.view(-1))
                # loss = lossfn(output, y_values)

                loss_clone = loss.clone()

                loss.backward()
                optimizer.step()                           

                loss_val = loss_clone.cpu().item() * y_values.size()[0]
                
                train_loss +=  loss_val

            model.eval()

            for batch in enumerate(valid_data_loader):
                x_values = batch[1]["x_values"].to(dtype=torch.float32).to(self.device)
                y_values = batch[1]["y_values"].to(dtype=torch.float32).to(self.device)

                output = model.forward(x_values)
                loss = lossfn.forward(output, y_values)                

                loss_val = loss.cpu().item() * y_values.size()[0]

                valid_loss += loss_val
            

            if self.interactive:
                print("{}/{} train loss: {} valid loss: {}".format(epoch, self.epochs, train_loss / num_training_samples, valid_loss / num_valid_samples))

            self.log_verbose("{}/{} train loss: {} valid loss: {}\n".format(epoch, self.epochs, train_loss / num_training_samples, valid_loss / num_valid_samples))
            train_loss = 0.
            valid_loss = 0.

            
            scheduler.step()     


        torch.save(model, self.model_path + "dynmodel{}.pth".format(self.run_id))
        torch.save(model.state_dict(), self.model_path + "dynmodel_states{}.pth".format(self.run_id))
        
        


if __name__ == "__main__":
    data_path = "/home/lbarnett/ros2_ws/src/rcraicer/rcraicer_mppi/training_data/"
    model_path = "/home/lbarnett/ros2_ws/src/rcraicer/rcraicer_mppi/models/"
    log_path = "/home/lbarnett/ros2_ws/src/rcraicer/rcraicer_mppi/logs/"
    dyn = Dyn(data_path, model_path, log_path, epochs=10, interactive=False)    
    # dyn = Dyn(data_path, model_path, log_path, epochs=10, interactive=True)    

    epochs = [500, 1000]
    lrs = [0.0001, 0.0002, 0.0003, 0.0004, 0.0005, 0.001]
    optims = ["sgd", "adam"]

    # epochs = [500]
    # lrs = [0.001] 
    # optims = ["sgd"]

    for epoch in epochs:
        for opt in optims:
            for lr in lrs:
                dyn.lr =lr
                dyn.optim = opt
                dyn.epochs = epoch
                dyn.learn()
                dyn.test()

                print("Completed run {}. lr={} epochs={} optim={}".format(dyn.run_id, lr, epoch, opt))