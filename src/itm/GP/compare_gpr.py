#!/usr/bin/env python
# coding=utf-8

'''
Date: 24.07.2021
Author: Yinfeng Long 
usage 
    python3 gpr_GPyTorch.py filename1.npz filename2.npz
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
x_train = torch. (gp_train['x_train']).flatten() ) # numpy into one dimension, then create a Tensor form from numpy (=torch.linspace) 
y_train = torch.from_numpy( (gp_train['y_train']).flatten() )
'''

# ### From .npz load datas for gp training### #
gp_train = np.load(sys.argv[1])
train_x = torch.from_numpy( (gp_train['arr_0']).flatten() )
train_y = torch.from_numpy( (gp_train['arr_1']).flatten() ) 
train_y += torch.randn(train_x.size()) * 0.01 #train_y += noise, noise ~ N(0,0.01)

train_x= (train_x).float()
train_y= (train_y).float()
print("train_x_type", train_x.type())
print("train_y_type", train_y.type())

data_driven = np.load(sys.argv[2])
data_driven_x =  (data_driven['arr_0']).flatten() 
data_driven_y =  (data_driven['arr_1']).flatten() 
print("data_driven_x_type", data_driven_x.shape )
print("data_driven_y_type", data_driven_y.shape )


# print
# print("gp_train[arr_1]: ", gp_train['arr_1'])
print("train_x: ", train_x)
print("train_y: ", train_y)
"""
# ### Prior theta Parameters ### #
l_scale = 0.1
sigma_f = 0.5
sigma_n = 0.01
"""

hypers = {
    'covar_module.base_kernel.lengthscale': torch.tensor(0.1),
    'covar_module.outputscale': torch.tensor(0.5),
    'likelihood.noise_covar.noise': torch.tensor(0.01)
}

# We will use the simplest form of GP model, exact inference
class ExactGPModel(gpytorch.models.ExactGP):
    def __init__(self, train_x, train_y, likelihood):
        super(ExactGPModel, self).__init__(train_x, train_y, likelihood)
        self.mean_module = gpytorch.means.ConstantMean()
        self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.RBFKernel())

    def forward(self, x):
        mean_x = self.mean_module(x)
        covar_x = self.covar_module(x)
        return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)

# initialize likelihood and model
likelihood = gpytorch.likelihoods.GaussianLikelihood()
model = ExactGPModel(train_x, train_y, likelihood)
model.initialize(**hypers)
model.train()
likelihood.train()

# Use the adam optimizer
optimizer = torch.optim.Adam([
    {'params': model.parameters()},  # Includes GaussianLikelihood parameters
], lr=0.1)

# "Loss" for GPs - the marginal log likelihood
mll = gpytorch.mlls.ExactMarginalLogLikelihood(likelihood, model)

training_iter = 2
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

    # Get into evaluation (predictive posterior) mode
model.eval()
likelihood.eval()

# Test points are regularly spaced along [-16,16]
# Make predictions by feeding model through likelihood
with torch.no_grad(), gpytorch.settings.fast_pred_var():
    test_x = ( torch.from_numpy( data_driven_x ) ).float()
    # test_x = torch.linspace(-16, 16, 100, dtype=torch.float)
    # test_x = train_x 
    t1 = time.time()
    observed_pred = likelihood(model(test_x))
    t2 = time.time()
    print("predict time of GPyTorch:", (t2-t1) )

with torch.no_grad():
    # Initialize plot
    f, ax = plt.subplots(1, 1, figsize=(4, 3))

    # Get upper and lower confidence bounds
    lower, upper = observed_pred.confidence_region()
    # Plot training data as black stars
    ax.plot(train_x.numpy(), train_y.numpy(), 'k*', alpha=0.1)
    # print("test_x: ", test_x.numpy())
    # print("test_x.numpy().shape: ", test_x.numpy().shape )
    # Plot predictive means as blue line
    ax.plot(data_driven_x, data_driven_y, 'g*')
    ax.plot(test_x.numpy(), observed_pred.mean.numpy(), 'r*')
    # ax.plot(test_x.numpy(), observed_pred.mean.numpy(), 'bs')
    # Shade between the lower and upper confidence bounds
    ax.fill_between(test_x.numpy(), lower.numpy(), upper.numpy(), alpha=0.5)
    ax.set_ylim([-21, 21])
    ax.legend(['Observed Data', 'predict_of_data_driven', 'predict_of_GPrTorch', 'confidence'])
    # ax.legend(['Observed Data', 'Mean', 'Confidence'])
    # print("observed_pred.mean.numpy(): ", observed_pred.mean.numpy())
    # print("observed_pred.mean.numpy().shape: ", observed_pred.mean.numpy().shape )
    
    '''
    f, ax2 = plt.subplots(1, 1, figsize=(4, 3))
    error = np.setdiff1d( observed_pred.mean.numpy(), data_driven_y) 
    ax2.plot(test_x.numpy()[:14000], error[:14000], 'g*')
    ax2.legend(['difference_of_mean'])
    '''
plt.show()
