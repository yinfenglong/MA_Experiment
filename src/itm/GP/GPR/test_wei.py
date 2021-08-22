#!/usr/bin/env python
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-08-21 21:33:38
LastEditors: Wei Luo
LastEditTime: 2021-08-21 23:06:48
Note: Note
'''
import torch
import gpytorch


import numpy as np
import os
from matplotlib import pyplot as plt


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
    file_path = os.path.dirname(os.path.realpath(__file__))
    data = np.load(file_path + '/for_gp_data.npz')

    if torch.cuda.is_available():
        device = torch.device("cuda")
    else:
        device = torch.device("cpu")
    # initialize likelihood and model
    likelihood = gpytorch.likelihoods.GaussianLikelihood().to(device)

    train_x = torch.from_numpy((data['arr_0']).flatten()).to(device)
    train_y = torch.from_numpy((data['arr_1']).flatten()).to(device)
    model = ExactGPModel(train_x, train_y, likelihood).to(device)

    model.train()
    likelihood.train()

    # Use the adam optimizer
    optimizer = torch.optim.Adam([
        # Includes GaussianLikelihood parameters
        {'params': model.parameters()},
    ], lr=0.1)

    # "Loss" for GPs - the marginal log likelihood
    mll = gpytorch.mlls.ExactMarginalLogLikelihood(
        likelihood, model).to(device)

    training_iter = 50
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

    # Test points are regularly spaced along [0,1]
    # Make predictions by feeding model through likelihood
    with torch.no_grad(), gpytorch.settings.fast_pred_var():
        test_x = torch.rand(31) * (12 + 11) - 11
        observed_pred = likelihood(model(test_x.to(device)))

    with torch.no_grad():
        # Initialize plot
        f, ax = plt.subplots(1, 1, figsize=(4, 3))

        # Get upper and lower confidence bounds
        lower, upper = observed_pred.confidence_region()

        train_x_cpu = train_x.cpu()
        train_y_cpu = train_y.cpu()
        test_x_cpu = test_x.cpu()
        # Plot training data as black stars
        ax.plot(train_x_cpu.numpy(), train_y_cpu.numpy(), 'k*')
        # Plot predictive means as blue line
        ax.plot(test_x_cpu.numpy(), observed_pred.mean.cpu().numpy(), 'r^')
        # Shade between the lower and upper confidence bounds
        # ax.fill_between(test_x_cpu.numpy(), lower.cpu().numpy(),
        #                 upper.cpu().numpy(), alpha=0.5)
        # ax.set_ylim([-3, 3])
        print(test_x_cpu.numpy().shape)
        print(observed_pred.mean.cpu().numpy().shape)
        print(np.concatenate((lower.cpu().numpy().reshape(-1, 1),
              upper.cpu().numpy().reshape(-1, 1)), axis=1).shape)
        # np.concatenate((lower.cpu().numpy()[0].reshape(-1, 1), upper.cpu().numpy()[0].reshape(-1, 1)), axis=1).reshape(2, -1)
        ax.errorbar(test_x_cpu.numpy(), observed_pred.mean.cpu().numpy(), yerr=np.concatenate(((-lower.cpu().numpy() +
                    observed_pred.mean.cpu().numpy()).reshape(1, -1), (upper.cpu().numpy() - observed_pred.mean.cpu().numpy()).reshape(1, -1)), axis=0), ecolor='b', elinewidth=2, capsize=4, fmt=' ')
        print(test_x_cpu.numpy())
        print(observed_pred.mean.cpu().numpy())
        print(lower.cpu().numpy())
        print(upper.cpu().numpy())
        ax.legend(['Observed Data', 'Mean', 'Confidence'])
        plt.show()
