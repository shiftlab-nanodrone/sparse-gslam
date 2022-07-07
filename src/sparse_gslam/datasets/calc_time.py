import numpy as np
import os
import sys
import matplotlib.pyplot as plt
import matplotlib

datasets = ["aces", "intel-lab", "mit-killian"]
# dataset = sys.argv[1]

# dtimes = []
ftimes = []
btimes = []
for dataset in datasets:
    ftimes.append(np.loadtxt(os.path.join(dataset, f"{dataset}.ftime")))
    btimes.append(np.loadtxt(os.path.join(dataset, f"{dataset}.btime")))

    print(
        dataset,
        np.average(np.diff(np.loadtxt(os.path.join(dataset, f"{dataset}.dtime")))),
        np.max(ftimes[-1]),
        np.max(btimes[-1]),
        (np.sum(ftimes[-1]) + np.sum(btimes[-1])) / len(ftimes[-1])
    )
    


fig1, ax1 = plt.subplots(figsize=(3.0, 2.2))
plt.subplots_adjust(left=0.2, bottom=0.2, right=0.99, top=0.99, wspace=0.3, hspace=None)
ax1.hist(ftimes, np.logspace(-4, -1.5, 40), stacked=True)
ax1.set_xscale("log")
f_ticks =[0.0001, 0.001, 0.01, 0.03]
ax1.set_xticks(f_ticks)
ax1.set_xticklabels([str(x) for x in f_ticks])
ax1.legend(["Aces", "Intel Lab", "MIT Killian"])
ax1.set_xlabel("Processing time (s)")
ax1.set_ylabel("Frequency")
plt.savefig("frontend.png")

fig1, ax1 = plt.subplots(figsize=(3.0, 2.2))
plt.subplots_adjust(left=0.2, bottom=0.2, right=0.99, top=0.99, wspace=0.3, hspace=None)
ax1.hist(btimes, np.logspace(-3, 0, 40), stacked=True)
ax1.set_xscale("log")
b_ticks =  [0.001, 0.01, 0.1, 0.7]
ax1.set_xticks(b_ticks)
ax1.set_xticklabels([str(x) for x in b_ticks])
# ax1.legend(["Aces", "Intel Lab", "MIT Killian"])
ax1.set_xlabel("Processing time (s)")
ax1.set_ylabel("Frequency")
plt.savefig("backend.png")
plt.show()