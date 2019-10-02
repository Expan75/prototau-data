import pandas as pd
import numpy as np
import seaborn as sns; sns.set()
import matplotlib.pyplot as plt
from sklearn import preprocessing

raw_df = pd.read_csv("data/FCLog_2019-07-02-113429.csv", header=1)
new_names = ['t','current','temp','voltage', 'pressure', 'state', 'alarm']
raw_df.columns = new_names

print(raw_df.head())
print(raw_df.info())

subset = raw_df.iloc[:14738]
skipSubset = subset.iloc[::175]

# Scales via normalisation
skipSubset['current'] = preprocessing.scale(skipSubset['current'])
skipSubset['temp'] = preprocessing.scale(skipSubset['temp'])
skipSubset['voltage'] = preprocessing.scale(skipSubset['voltage'])
skipSubset['pressure'] = preprocessing.scale(skipSubset['pressure'])

data = skipSubset[['t','current','temp','voltage','pressure']]
print(data)



"""
fig, ax = plt.subplots(1, 1)

ax.spines['top'].set_visible(False)
ax.spines['bottom'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['left'].set_visible(False)

ax.get_xaxis().tick_bottom()
ax.get_yaxis().tick_left()
plt.tick_params(
    axis='x',          # changes apply to the x-axis
    which='both',      # both major and minor ticks are affected
    bottom=False,      # ticks along the bottom edge are off
    top=False,         # ticks along the top edge are off
    labelbottom=False) # labels along the bottom edge are off

color_sequence = ['#1f77b4', '#aec7e8', '#ff7f0e', '#ffbb78']
dataCols = ['current','temp','voltage','pressure']

for colIndex, colName in enumerate(dataCols):
    line = plt.plot(data.t,
                data[colName],
                lw=2.2,
                color=color_sequence[colIndex],
                label=colName)

legend = ax.legend(loc='upper right', shadow=False, fontsize='medium')

plt.show()
"""