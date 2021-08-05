#!/usr/bin/env python
# coding=utf-8

'''
Date: 03.08.2021
Author: Yinfeng Long
usage
    python3 gpr_GPyTorch_predict.py file_name.npz
'''

import sys
import numpy as np
import joblib
import torch
import gpytorch
from matplotlib import pyplot as plt
import datetime
import time

'''
# ### From .pkl load datas ### #
gp_train = joblib.load(sys.argv[1])
# numpy into one dimension, then create a Tensor form from numpy (=torch.linspace)
x_train = torch.from_numpy( (gp_train['x_train']).flatten() )
y_train = torch.from_numpy( (gp_train['y_train']).flatten() )
'''

# if torch.cuda.is_available():
#     device = torch.device("cuda")
# else:
#     device = torch.device("cpu")

device = torch.device("cpu")

# # ### From .npz load datas for gp training### #
gp_train = np.load(sys.argv[1])
print(gp_train['arr_0'].shape)
# numpy into one dimension, then create a Tensor form from numpy (=torch.linspace)
train_x = torch.from_numpy((gp_train['arr_0'][:5000]).flatten())
train_y = torch.from_numpy(
    (gp_train['arr_1'][:5000]).flatten())  # numpy into one
# train_y += noise, noise ~ N(0,0.01)
train_y += torch.randn(train_x.size()) * 0.01

train_x = (train_x).float().to(device)
train_y = (train_y).float().to(device)


##############################
# prediction #
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

# if not torch.cuda.is_available():
#     device = torch.device("cpu")
#     target_device = 'cpu'
# else:
#     device = torch.device("cuda")
#     target_device = 'cuda:0'

target_device = 'cuda:0'

likelihood_pred = gpytorch.likelihoods.GaussianLikelihood()
model_to_predict = ExactGPModel(train_x, train_y, likelihood_pred).to(target_device)

if torch.cuda.is_available():
    model_state_dict = torch.load('./model_state.pth')
    likelihood_state_dict = torch.load('./likelihood_state.pth')
else:
    model_state_dict = torch.load('./model_state.pth', map_location=target_device)
    likelihood_state_dict = torch.load('./likelihood_state.pth', map_location=target_device)
model_to_predict.load_state_dict(model_state_dict)
likelihood_pred.load_state_dict(likelihood_state_dict)

# model_to_predict = ExactGPModel(train_x, train_y, likelihood).to(target_device)
# model_to_predict.load_state_dict(torch.load(
#     './best.pth', map_location=target_device))
# likelihood_pred = gpytorch.likelihoods.GaussianLikelihood().to(target_device)

# model_to_predict = model
# likelihood_pred = likelihood
model_to_predict.eval()
likelihood_pred.eval()

# Test points are regularly spaced along [-16,16]
# Make predictions by feeding model through likelihood
# test_x = torch.linspace(-16, 16, 10, dtype=torch.float).to(target_device)
test_x = train_x.to(target_device)
with torch.no_grad(), gpytorch.settings.fast_pred_var():
    # test_x = train_x
    t1 = time.time()
    # t1 = datetime.datetime.now()
    # for i in range(10):
    observed_pred = likelihood_pred(model_to_predict(test_x))
    t2 = time.time()
    # t2 = datetime.datetime.now()
    print("predict time of GPyTorch (predict 100 new numbers):",
          (t2 - t1))
    # print("predict time of GPyTorch (predict 100 new numbers):",
        #   (t2 - t1).microseconds)
    
    ############
    # select gp
    ############
    sum_distance = np.sum( np.sqrt( ( observed_pred.mean.cpu().numpy() - 0.10001323  )**2 )   )
    print("sum of distance:",
          sum_distance )
    
with torch.no_grad():
    # Initialize plot
    f, ax = plt.subplots(1, 1, figsize=(4, 3))

    # Get upper and lower confidence bounds
    lower, upper = observed_pred.confidence_region()
    train_x_cpu = train_x.cpu()
    train_y_cpu = train_y.cpu()
    if target_device == 'cuda:0':
        test_x_cpu = test_x.cpu()
    else:
        test_x_cpu = test_x
    # Plot training data as black stars
    ax.plot(train_x_cpu.numpy(), train_y_cpu.numpy(), 'k*', alpha=0.1)
    # print("test_x: ", test_x.numpy())
    # print("test_x.numpy().shape: ", test_x.numpy().shape )
    # Plot predictive means as blue line
    ax.plot(test_x_cpu.numpy(), observed_pred.mean.cpu().numpy(), 'r*')
    # ax.plot(test_x.numpy(), observed_pred.mean.numpy(), 'bs')
    # Shade between the lower and upper confidence bounds
    ax.fill_between(test_x_cpu.numpy(), lower.cpu().numpy(),
                    upper.cpu().numpy(), alpha=0.5)
    ax.set_ylim([-21, 21])
    ax.legend(['Observed Data', 'Mean', 'Confidence'])
    # print("observed_pred.mean.numpy(): ", observed_pred.mean.numpy())
    # print("observed_pred.mean.numpy().shape: ", observed_pred.mean.numpy().shape )
plt.show()
