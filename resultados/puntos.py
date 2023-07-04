import pandas
import numpy
import math
# importing matplotlib module
import matplotlib.pyplot as plt
import matplotlib
plt.style.use('default')
# import scipy module
import scipy.signal

x = [2,-2]
y = [0,0]

fig = plt.figure(figsize=(16, 10), dpi=150)

plt.plot(x,y,marker="o", markersize=10, markeredgecolor="y", markerfacecolor="green",linestyle = 'none')

plt.show()