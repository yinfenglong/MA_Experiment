#!/usr/bin/env python
# coding=utf-8

'''
Date: 03.08.2021
Author: Yinfeng Long
usage
    python3 gpr_GPyTorch.py filename.npz
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
time_1 = time.time()
# ### From .npz load datas for gp training### #
gp_train = np.load(sys.argv[1])
print(gp_train['arr_0'].shape)
# numpy into one dimension, then create a Tensor form from numpy (=torch.linspace)
# train_x = torch.from_numpy((gp_train['arr_0'][:5000]).flatten())
train_x = torch.from_numpy((gp_train['arr_0']).flatten())
train_y = torch.from_numpy(
    # (gp_train['arr_1'][:5000]).flatten())  # numpy into one
    (gp_train['arr_1']).flatten())  # numpy into one
# train_y += noise, noise ~ N(0,0.01)
train_y += torch.randn(train_x.size()) * 0.01

train_x = (train_x).float().to(device)
train_y = (train_y).float().to(device)

# print
print("gp_train[arr_1]: ", gp_train['arr_1'])
print("train_x: ", train_x)
print("train_y: ", train_y)
"""
# ### Prior theta Parameters ### #
l_scale = 0.1
sigma_f = 0.5
sigma_n = 0.01
"""

hypers = {
    'covar_module.base_kernel.lengthscale': torch.tensor(0.1).to(device),
    'covar_module.outputscale': torch.tensor(0.5).to(device),
    'likelihood.noise_covar.noise': torch.tensor(0.01).to(device),
}

# We will use the simplest form of GP model, exact inference


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


# initialize likelihood and model
likelihood = gpytorch.likelihoods.GaussianLikelihood().to(device)
model = ExactGPModel(train_x, train_y, likelihood).to(device)
model.initialize(**hypers)
model.train()
likelihood.train()

# Use the adam optimizer
optimizer = torch.optim.Adam([
    {'params': model.parameters()},  # Includes GaussianLikelihood parameters
], lr=0.1)

# "Loss" for GPs - the marginal log likelihood
mll = gpytorch.mlls.ExactMarginalLogLikelihood(likelihood, model)

training_iter = 10
for i in range(training_iter):
    # Zero gradients from previous iteration
    optimizer.zero_grad()
    # Output from model
    output = model(train_x)
    # Calc loss and backprop gradients
    loss = -mll(output, train_y)
    loss.backward()
    print('Iter %d/%d - Loss: %.3f   lengthscale: %.3f   noise: %.3f' % (
        i + 1, training_iter, loss.item(),
        model.covar_module.base_kernel.lengthscale.item(),
        model.likelihood.noise.item()
    ))
    optimizer.step()

time_2 = time.time()
print("training time is: ", (time_2 - time_1))
torch.save(model.state_dict(), './model_state.pth')
likelihood_state_dict = likelihood.state_dict()
torch.save(likelihood_state_dict, './likelihood_state.pth')

