import matplotlib.pyplot as plt
import os

def get_plots():
    fig, axes = plt.subplots(1, 2, figsize=(6.75, 2.5))
    plt.subplots_adjust(left=0.09, bottom=0.2, right=0.99, top=None, wspace=0.3, hspace=None)
    return fig, axes


def read_errors(dname, xs):
    rot_ys = []
    rot_stds = []
    trans_ys = []
    trans_stds = []
    for i in xs:    
        with open(os.path.join(dname, f"{dname}-{i}_rot_error.log")) as f:
            line = f.readlines()[1].split(", ")[:2]
            rot_ys.append(float(line[0]))
            rot_stds.append(float(line[1]) / 2)
        with open(os.path.join(dname, f"{dname}-{i}_trans_error.log")) as f:
            line = f.readlines()[1].split(", ")[:2]
            trans_ys.append(float(line[0]))
            trans_stds.append(float(line[1]) / 2)
    return rot_ys, rot_stds, trans_ys, trans_stds


def plot_errors(dname: str):
    # xs = [4, 5, 6, 7, 8, 10, 11, 13, 30, 45, 60]
    xs = [4, 6, 9, 11, 13, 30, 45, 60]
    g_xs = [30, 45, 60]
    rot_ys, rot_stds, trans_ys, trans_stds = read_errors(dname, xs)
    g_rot_ys, g_rot_stds, g_trans_ys, g_trans_stds = read_errors(dname, [f"gmapping-{x}" for x in g_xs])

    fig, axes = get_plots()
    axes[0].plot(xs, rot_ys)
    axes[0].plot(g_xs, g_rot_ys)
    axes[0].scatter(xs, rot_ys, marker="*", s=64)
    axes[0].scatter(g_xs, g_rot_ys, marker="*", s=64)
    axes[0].set_xlabel("Number of Range Measurements")
    axes[0].set_ylabel("Error (degrees)")
    axes[0].legend(["Ours", "GMapping"], loc="upper right")
    axes[0].set_title("Absolute Rotational Error", fontsize=11)

    axes[1].plot(xs, trans_ys)
    axes[1].plot(g_xs, g_trans_ys)
    axes[1].scatter(xs, trans_ys, marker="*", s=64)
    axes[1].scatter(g_xs, g_trans_ys, marker="*", s=64)
    axes[1].legend(["Ours", "GMapping"], loc="upper right")
    axes[1].set_xlabel("Number of Range Measurements")
    axes[1].set_ylabel("Error (meter)")
    axes[1].set_title("Absolute Translational Error", fontsize=11)
    plt.savefig("sweep.PNG")

    xs = ["4-k0", "4", "4-k5", "4-k7"]
    rot_ys, rot_stds, trans_ys, trans_stds = read_errors(dname, xs)

    fig, axes = get_plots()
    xs = ["no kernel", "3x3", "5x5", "7x7"]
    x = list(range(len(xs)))
    axes[0].plot(x, rot_ys)
    axes[0].scatter(x, rot_ys, marker="*", s=64)
    axes[0].set_xlabel("Kernel choices")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(xs)
    axes[0].set_ylabel("Error (degrees)")
    axes[0].set_title("Absolute Rotational Error", fontsize=11)

    axes[1].plot(x, trans_ys)
    axes[1].scatter(x, trans_ys, marker="*", s=64)
    axes[1].set_xlabel("Kernel choices")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(xs)
    axes[1].set_ylabel("Error (meter)")
    axes[1].set_title("Absolute Translational Error", fontsize=11)
    plt.savefig("kernel.PNG")

    xs = list(range(92, 136, 4))
    xs.remove(124)
    rot_ys, rot_stds, trans_ys, trans_stds = read_errors(dname, [f"4-m-{x}" for x in xs])

    fig, axes = get_plots()
    axes[0].plot(xs, rot_ys)
    axes[0].scatter(xs, rot_ys, marker="*", s=64)
    axes[0].set_xlabel("Multiscan size")
    axes[0].set_ylabel("Error (degrees)")
    axes[0].set_title("Absolute Rotational Error", fontsize=11)

    axes[1].plot(xs, trans_ys)
    axes[1].scatter(xs, trans_ys, marker="*", s=64)
    axes[1].set_xlabel("Multiscan size")
    axes[1].set_ylabel("Error (meter)")
    axes[1].set_title("Absolute Translational Error", fontsize=11)
    plt.savefig("multiscan.PNG")
    plt.show()

# plot_errors("aces")
plot_errors("intel-lab")