#!/usr/bin/env python
# coding=utf-8

'''
Date: 24.07.2021
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

if torch.cuda.is_available():
    device = torch.device("cuda")
else:
    device = torch.device("cpu")

# ### From .npz load datas for gp training### #
gp_train = np.load(sys.argv[1])
print(gp_train['arr_0'].shape)
# numpy into one dimension, then create a Tensor form from numpy (=torch.linspace)
train_x = torch.from_numpy((gp_train['arr_0'][:5000]).flatten())
train_y = torch.from_numpy(
    (gp_train['arr_1'][:5000]).flatten())  # numpy into one
# train_y += noise, noise ~ N(0,0.01)
train_y += torch.randn(train_x.size()) * 0.01

train_x = (train_x).float().cuda()
train_y = (train_y).float().cuda()

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
    'covar_module.base_kernel.lengthscale': torch.tensor(0.1).cuda(),
    'covar_module.outputscale': torch.tensor(0.5).cuda(),
    'likelihood.noise_covar.noise': torch.tensor(0.01).cuda(),
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
likelihood = gpytorch.likelihoods.GaussianLikelihood().cuda()
model = ExactGPModel(train_x, train_y, likelihood).cuda()
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

    # Get into evaluation (predictive posterior) mode
torch.save(model.state_dict(), './best.pth')
torch.save(likelihood, './best_l.pth')

##############################
# prediction #
##############################

if not torch.cuda.is_available():
    device = torch.device("cpu")
    target_device = 'cpu'
else:
    device = torch.device("cuda")
    target_device = 'cuda:0'


# likelihood_to_predict = gpytorch.likelihoods.GaussianLikelihood()
# print("dfddfdfd")
# likelihood_to_predict.load_state_dict(
#     torch.load('./best_l.pth', map_location=target_device))
print("dfddfdfd")
model_to_predict = ExactGPModel(train_x, train_y, likelihood).to(target_device)
model_to_predict.load_state_dict(torch.load(
    './best.pth', map_location=target_device))

model_to_predict.eval()
likelihood.eval()

# Test points are regularly spaced along [-16,16]
# Make predictions by feeding model through likelihood
test_x = torch.linspace(-16, 16, 100, dtype=torch.float).to(device)
with torch.no_grad(), gpytorch.settings.fast_pred_var():
    # test_x = train_x
    t1 = time.time()
    observed_pred = likelihood(model_to_predict(test_x))
    t2 = time.time()
    print("predict time of GPyTorch (predict 100 new numbers):",
          (t2 - t1))

with torch.no_grad():
    # Initialize plot
    f, ax = plt.subplots(1, 1, figsize=(4, 3))

    # Get upper and lower confidence bounds
    lower, upper = observed_pred.confidence_region()
    train_x_cpu = train_x.cpu()
    train_y_cpu = train_y.cpu()
    test_x_cpu = test_x.cpu()
    # Plot training data as black stars
    ax.plot(train_x_cpu.numpy(), train_y_cpu.numpy(), 'k*', alpha=0.1)
    # print("test_x: ", test_x.numpy())
    # print("test_x.numpy().shape: ", test_x.numpy().shape )
    # Plot predictive means as blue line
    ax.plot(test_x_cpu.numpy(), observed_pred.mean.cpu().numpy(), 'r')
    # ax.plot(test_x.numpy(), observed_pred.mean.numpy(), 'bs')
    # Shade between the lower and upper confidence bounds
    ax.fill_between(test_x_cpu.numpy(), lower.cpu().numpy(),
                    upper.cpu().numpy(), alpha=0.5)
    ax.set_ylim([-21, 21])
    ax.legend(['Observed Data', 'Mean', 'Confidence'])
    # print("observed_pred.mean.numpy(): ", observed_pred.mean.numpy())
    # print("observed_pred.mean.numpy().shape: ", observed_pred.mean.numpy().shape )
plt.show()
