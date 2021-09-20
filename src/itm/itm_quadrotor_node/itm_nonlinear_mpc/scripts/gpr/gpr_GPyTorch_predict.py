#!/usr/bin/env python3
# coding=utf-8

'''
Date: 19.09.2021
Author: Yinfeng Long
usage
    x_train_idx = ['x', 'y', 'z', 'vx', 'vy', 'vz']
    y_train_idx = ['y_x', 'y_y', 'y_z', 'y_vx', 'y_vy', 'y_vz']
'''

import numpy as np
import torch
import gpytorch
import sys
from matplotlib import pyplot as plt
import time
import os.path

class GpMean(object):
    def __init__(self, x_train_idx, y_train_idx, gp_model_file_path, npz_name):
        self.file_path = gp_model_file_path
        self.npz_name = npz_name

        self.train_x = None
        self.train_x_max = None
        self.train_x_min = None
        self.train_y = None
        self.train_y_range = None
        self.load_data(x_train_idx, y_train_idx)
        self.model_to_predict = None
        self.likelihood_pred = None
        self.load_model(x_train_idx)

        self.observed_pred = None
        self.test_x = None
    
    def load_data(self, x_train_idx, y_train_idx):
        gp_train = np.load(self.file_path + '/' + self.npz_name)

        if torch.cuda.is_available():
            device = torch.device("cuda")
        else:
            device = torch.device("cpu")

        train_x_ori = (gp_train[x_train_idx]).flatten()
        train_y_ori = (gp_train[y_train_idx]).flatten()
        self.train_x_max = np.max(train_x_ori)
        self.train_x_min = np.min(train_x_ori)
        print("train_x_max:", self.train_x_max)
        print("train_x_min:", self.train_x_min)
        self.train_y_range = np.max(train_y_ori) - np.min(train_y_ori)

        self.train_x = torch.from_numpy( train_x_ori[:7000] )
        self.train_y = torch.from_numpy( train_y_ori[:7000])

        # numpy into one dimension, then create a Tensor form from numpy (=torch.linspace)
        print("x_train shape", self.train_x.shape)
        print("y_train shape", self.train_y.shape)
        self.train_x = (self.train_x).float().to(device)
        self.train_y = (self.train_y).float().to(device)
    
    def load_model(self, x_train_idx):
        target_device = 'cuda:0'

        self.likelihood_pred = gpytorch.likelihoods.GaussianLikelihood()
        self.model_to_predict = ExactGPModel(self.train_x, self.train_y, self.likelihood_pred).to(target_device)

        if torch.cuda.is_available():
            model_state_dict = torch.load( self.file_path + '/train_pre_model/model_state_' + x_train_idx +'.pth')
            likelihood_state_dict = torch.load(self.file_path + '/train_pre_model/likelihood_state_' + x_train_idx +'.pth')
        else:
            model_state_dict = torch.load(self.file_path + '/train_pre_model/model_state_' + x_train_idx +'.pth', map_location=target_device)
            likelihood_state_dict = torch.load(self.file_path + '/train_pre_model/likelihood_state_' + x_train_idx+'.pth', map_location=target_device)
        self.model_to_predict.load_state_dict(model_state_dict)
        self.likelihood_pred.load_state_dict(likelihood_state_dict)

        self.model_to_predict.eval()
        self.likelihood_pred.eval()

    def predict_test(self, ):
        target_device = 'cuda:0'

        self.test_x = torch.rand(51) * (self.train_x_max - self.train_x_min) + self.train_x_min
        with torch.no_grad(), gpytorch.settings.fast_pred_var():
            # test_x = train_x
            t1 = time.time()
            # t1 = datetime.datetime.now()
            self.observed_pred = self.likelihood_pred(self.model_to_predict(self.test_x.to(target_device)))
            t2 = time.time()
            # t2 = datetime.datetime.now()
            print("predict time of GPyTorch (predict 100 new numbers):",
                (t2 - t1))

    def plot_predict_result(self,):
        target_device = 'cuda:0'
        with torch.no_grad():
            # Initialize plot
            f, ax = plt.subplots(1, 1, figsize=(4, 3), dpi=150)

            # Get upper and lower confidence bounds
            lower, upper = self.observed_pred.confidence_region()
            train_x_cpu = self.train_x.cpu()
            train_y_cpu = self.train_y.cpu()
            if target_device == 'cuda:0':
                test_x_cpu = self.test_x.cpu()
            else:
                test_x_cpu = self.test_x
            # Plot training data as black stars
            ax.plot(train_x_cpu.numpy(), train_y_cpu.numpy(), 'k*', alpha=0.5)
            observed_pred_np = self.observed_pred.mean.cpu().numpy()
            ax.plot(test_x_cpu.numpy(), observed_pred_np, 'r*')

            print("observed_pred.mean.numpy(): ", observed_pred_np)
            print("observed_pred.mean.numpy().shape: ", observed_pred_np.shape )
            ax.errorbar(test_x_cpu.numpy(), observed_pred_np, yerr=np.concatenate(((-lower.cpu().numpy() +
                        observed_pred_np).reshape(1, -1), (upper.cpu().numpy() - observed_pred_np).reshape(1, -1)), axis=0), ecolor='b', elinewidth=2, capsize=4, fmt=' ')
            ax.legend(['Observed Data', 'Mean', 'Confidence'])

            if self.train_y_range < 2:
                maloc = 0.05 
                miloc = 0.05
            else:
                # maloc = float( '%.1f'%(train_y_range/30))
                # miloc = maloc / 2
                maloc = 1 
                miloc = 0.5
            print("maloc:", maloc)
            print("train_y_range:", self.train_y_range)
            ax.yaxis.set_major_locator( plt.MultipleLocator(maloc) )
            ax.yaxis.set_minor_locator( plt.MultipleLocator(miloc) )
            ax.grid(axis='y', which='major', color='darkorange', alpha=1)
            ax.grid(axis='y', which='minor', color='darkorange', alpha=0.5)

            plt.title( sys.argv[1] + '/' + x_train_idx )
            # manger = plt.get_current_fig_manager()
            # manger.window.showMaximized()
            fig = plt.gcf()
            plt.show()
            figures_path = './' + sys.argv[1] + '/figures/'
            if not os.path.exists(figures_path):
                os.makedirs( figures_path )
            fig.savefig( figures_path + x_train_idx + '.png' )

    def predict_mean(self, test_point):
        target_device = 'cuda:0'

        test_x = torch.tensor(test_point).float().to(target_device)
        with torch.no_grad(), gpytorch.settings.fast_pred_var():
            observed_pred = self.likelihood_pred(self.model_to_predict(test_x))
            return observed_pred.mean.cpu().numpy()

##############################
# GpyTorch #
##############################
class ExactGPModel(gpytorch.models.ExactGP):
    def __init__(self, train_x, train_y, likelihood):
        super(ExactGPModel, self).__init__(train_x, train_y, likelihood)
        self.mean_module = gpytorch.means.ConstantMean()
        self.covar_module = gpytorch.kernels.ScaleKernel(
            gpytorch.kernels.RBFKernel())

    def forward(self, x):
        mean_x = self.mean_module(x)
        covar_x = self.covar_module(x)
        return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)

if __name__ == '__main__':
    # ### From .npz load datas for gp training### #
    file_path = './' + sys.argv[1]
    npz_name = 'datas_for_gp_y.npz'
    # npz_name = sys.argv[2] 
    gp_train = np.load( file_path + '/' + npz_name)

    x_idx_list = [i for i in gp_train.keys()][:6]
    y_idx_list = [i for i in gp_train.keys()][6:]
    for i in range(len(x_idx_list)):
        x_train_idx = x_idx_list[i]
        y_train_idx = y_idx_list[i]
        print("***************************")
        print("x_train_idx: {}".format(x_train_idx) )
        print("y_train_idx: {}".format(y_train_idx) )

        gpMPC = GpMean(x_train_idx, y_train_idx, file_path, npz_name)
        gpMPC.predict_test()
        gpMPC.plot_predict_result()

    # test one point
    # file_path ="/home/itm_stud/test_ma_ws/MA_Experiment/src/itm/itm_quadrotor_node/itm_nonlinear_mpc/scripts/gpr/q330"
    # npz_name ="combined_q330.npz"
    # gpMPCVx = GpMeanCombine('vx', 'y_vx', file_path, npz_name)

    # t1 = time.time()
    # a = np.array([4.68015e-310])
    # x_1 = gpMPCVx.predict_mean(a)
    # print("x1: {}".format(x_1))
    # t2 = time.time()
    # print("first training time is: ", (t2 - t1))