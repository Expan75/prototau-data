import pandas as pd
import numpy as np

raw_df = pd.read_csv("data/FCLog_2019-07-02-113429.csv", header=1)
new_names = ['t','current','temp','voltage', 'pressure', 'state', 'alarm']
raw_df.columns = new_names

print(raw_df.head())
print(raw_df.info())

subset = raw_df.iloc[:14738]
skipSubset = subset.iloc[::50]
print(subset)

import seaborn as sns; sns.set()
import matplotlib.pyplot as plt

# rename column names

ax = sns.lineplot(x=skipSubset.t.astype(int), y=skipSubset.pressure.astype(float))
plt.show()