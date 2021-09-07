"""
This module is used to change the format for a given matplotlib figure for
different purposes. Similar to the itm_formatfig.m file for Matlab.

It contains the function formatfig to do that.

Usage example:
--------------------------------------------
from matplotlib_formatfig import set_rcParams
import matplotlib.pyplot as plt

set_rcParams('latex')
plt.plot([0, 2, 3])

plt.savefig('myplot.pdf')
plt.show()
--------------------------------------------

Author: Christian Fischer <cfischer@itm.uni-stuttgart.de>
"""

from matplotlib.pyplot import rcParams, rcdefaults

# -----------------------------------------------------------------------------
def set_rcParams(preset='beamer', size=None):
    """This function changes the rcParams with the given preset."""
    
    if preset in ('beamer','PP'):
        if size == None:
            size = [5.9, 5.9]
        rcParams.update({
            'axes.labelsize': 26,
            'axes.linewidth': 2,
            'axes.grid' : True,
            'grid.linewidth': 1.6,
            'grid.linestyle': 'dotted',
            'text.fontsize': 30,
            'xtick.labelsize': 20,
            'ytick.labelsize': 20,
            'legend.fontsize': 26,
            #'legend.pad' : 0.2,   # empty space around the legend box
            'lines.markersize': 10,
            'lines.linewidth': 2.0,
            'font.size': 26,
            #'font.family': 'serif',
            #'font.serif' : 'Computer Modern Roman',
            'mathtext.default': 'sf',
            #'text.usetex': True,
            'text.latex.unicode': True,
            'savefig.dpi': 150,
            'figure.figsize': size,
            #'figure.dpi' : 72,
            'figure.subplot.bottom': 0.12,
            #'figure.subplot.hspace': 0.2,
            'figure.subplot.left': 0.20,
            'figure.subplot.right': 0.95,
            'figure.subplot.top': 0.95,
            #'figure.subplot.wspace': 0.2,
        })

                  
    elif preset == 'latex':
        if size == None:
            size = [5.4, 3.2]
        rcParams.update({
            'axes.labelsize': 12,
            'axes.linewidth': 0.5,
            #'grid.linewidth': 1.6,
            #'grid.linestyle': 'dotted',
            # 'text.fontsize': 12,
            'xtick.labelsize': 10,
            'ytick.labelsize': 10,
            #'legend.pad': 0.2,     # empty space around the legend box
            'legend.fontsize': 12,
            #'lines.markersize': 10,
            #'lines.linewidth': 2,
            'font.size': 12,
            'font.family': 'serif',
            'font.serif' : 'Computer Modern Roman',
            'text.usetex': True,
            'text.latex.unicode': True,
            'text.latex.preamble': [r"\usepackage{amstext}",
                                    r"\usepackage{units}"],
            'figure.figsize': size,
            'figure.subplot.bottom': 0.14,
            #'figure.subplot.hspace': 0.2,
            'figure.subplot.left': 0.12,
            'figure.subplot.right': 0.95,
            'figure.subplot.top': 0.95,
            'text.usetex': True,
            #'figure.subplot.wspace': 0.2,
        })

    elif preset == 'latexnarrow':
        if size == None:
            size = [2.8, 2.8]
        rcParams.update({
            'axes.labelsize': 12,
            'axes.linewidth': 0.5,
            #'grid.linewidth': 1.6,
            #'grid.linestyle': 'dotted',
            'text.fontsize': 12,
            'xtick.labelsize': 10,
            'ytick.labelsize': 10,
            #'legend.pad': 0.2,     # empty space around the legend box
            'legend.fontsize': 12,
            #'lines.markersize': 10,
            #'lines.linewidth': 2,
            'font.size': 12,
            'font.family': 'serif',
            'font.serif' : 'Computer Modern Roman',
            'text.usetex': True,
            'text.latex.unicode': True,
            'text.latex.preamble': [r"\usepackage{amstext}",
                                    r"\usepackage{units}"],
            'figure.figsize': size,
            'figure.subplot.bottom': 0.14,
            #'figure.subplot.hspace': 0.2,
            'figure.subplot.left': 0.12,
            'figure.subplot.right': 0.95,
            'figure.subplot.top': 0.95,
            #'figure.subplot.wspace': 0.2,
        })

    elif preset =='reset':
        rcdefaults()

# -----------------------------------------------------------------------------
def formatfig(fig, preset='beamer', size=(5.9, 5.9) ):
    """This function adjusts the format of a given figure with the given preset.
    """
    params = rcParams.copy()
    
    if preset in ('beamer','PP'):
        params.update({'axes.labelsize': 26,
                  'axes.linewidth': 2,
                  'grid.linewidth': 1.6,
                  'grid.linestyle': 'dotted',
                  'text.fontsize': 26,
                  'xtick.labelsize': 20,
                  'ytick.labelsize': 20,
                  #'legend.pad': 0.2,     # empty space around the legend box
                  'legend.fontsize': 26,
                  'lines.markersize': 10,
                  'lines.linewidth': 2,
                  'font.size': 26})
#                  'text.usetex': True})
                  #'figure.figsize': get_figsize(800)}
        fig.set_size_inches(size)
        fig.set_dpi(72)
        fig.subplots_adjust(left=0.18, bottom=0.14, right=0.95, top=0.95)
        rcParams.update({"savefig.dpi": 150})
        
    elif preset =='latex':
        params.update({'axes.labelsize': 12,
                  'axes.linewidth': 0.5,
                  #'grid.linewidth': 1.6,
                  #'grid.linestyle': 'dotted',
                  'text.fontsize': 12,
                  'xtick.labelsize': 10,
                  'ytick.labelsize': 10,
                  #'legend.pad': 0.2,     # empty space around the legend box
                  'legend.fontsize': 12,
                  #'lines.markersize': 10,
                  #'lines.linewidth': 2,
                  'font.size': 12,
                  'font.family': 'serif'})
        fig.set_size_inches(5.4, 3.2)
        fig.subplots_adjust(left=0.1, bottom=0.14, right=0.97, top=0.95)
        rcParams['font.serif'] = 'Computer Modern Roman'
        rcParams['font.family'] = 'serif'

    # iterate over the axes objects (subplots) of fig and set the parameters
    for axes in fig.axes:
        axes.xaxis.get_label().set_size(params['axes.labelsize'])
        axes.yaxis.get_label().set_size(params['axes.labelsize'])
        axes.get_frame().set_linewidth(params['axes.linewidth'])
        # iterate over the line objects (plots) of the current subplot
        for line in axes.lines:
            if line.get_linewidth() < params['lines.linewidth']:
                line.set_linewidth(params['lines.linewidth'])
            if line.get_markersize() < params['lines.markersize']:
                line.set_markersize(params['lines.markersize'])
        # iterate over the vertical gridline objects
        for line in axes.xaxis.get_gridlines():
            line.set_linewidth(params['grid.linewidth'])
            line.set_linestyle(params['grid.linestyle'])
        # iterate over the horizontal gridline objects
        for line in axes.yaxis.get_gridlines():
            line.set_linewidth(params['grid.linewidth'])
            line.set_linestyle(params['grid.linestyle'])
        # iterate over the xticklabel objects
        for ticklbl in axes.xaxis.get_ticklabels():
            ticklbl.set_fontsize(params['xtick.labelsize'])
            ticklbl.get_fontproperties().set_family(params['font.family'])
        # iterate over the yticklabel objects
        for ticklbl in axes.yaxis.get_ticklabels():
            ticklbl.set_fontsize(params['ytick.labelsize'])
            ticklbl.get_fontproperties().set_family(params['font.family'])
        # iterate over all text objects in legend
        if not axes.get_legend() == None:
            for text in axes.get_legend().texts:
                text.set_fontsize(params['legend.fontsize'])
                text.get_fontproperties().set_family(params['font.family'])
        # iterate over all text objects
        for text in axes.texts:
            text.set_fontsize(params['text.fontsize'])
            
# -----------------------------------------------------------------------------
def get_properties(fig):
    figsize = fig.get_size_inches()
    figdpi = fig.get_dpi()
    subplotpars = fig.subplotpars
    savefig_dpi = rcParams.get("savefig.dpi")

    # iterate over the axes objects (subplots) of fig
    for axes in fig.axes:
        axes.xaxis.get_label().set_size(params['axes.labelsize'])
        axes.yaxis.get_label().set_size(params['axes.labelsize'])
        axes.get_frame().set_linewidth(params['axes.linewidth'])
        # iterate over the line objects (plots) of the current subplot
        for line in axes.lines:
            line.set_linewidth(params['lines.linewidth'])
            line.set_markersize(params['lines.markersize'])
        # iterate over the vertical gridline objects
        for line in axes.xaxis.get_gridlines():
            line.set_linewidth(params['grid.linewidth'])
            line.set_linestyle(params['grid.linestyle'])
        # iterate over the horizontal gridline objects
        for line in axes.yaxis.get_gridlines():
            line.set_linewidth(params['grid.linewidth'])
            line.set_linestyle(params['grid.linestyle'])
        # iterate over the xticklabel objects
        for ticklbl in axes.xaxis.get_ticklabels():
            ticklbl.set_fontsize(params['xtick.labelsize'])
        # iterate over the yticklabel objects
        for ticklbl in axes.yaxis.get_ticklabels():
            ticklbl.set_fontsize(params['ytick.labelsize'])
        # iterate over all text objects in legend
        for text in axes.get_legend().texts:
            text.set_fontsize(params['legend.fontsize'])
        # iterate over all text objects
        for text in axes.texts:
            text.set_fontsize(params['text.fontsize'])
#

