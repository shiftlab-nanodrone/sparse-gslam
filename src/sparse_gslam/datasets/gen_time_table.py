def get_our_time(fname):
    c = open(f"{fname}/{fname}.time").readlines()[1].split(",")
    return float(c[0]), float(c[1]) # , float(c[2]) * 1000

table = """
\\begin{tabular}{@{}lllll@{}}
\\toprule
Dataset        & Data duration   & Wall clock  & GMapping & Cartographer  \\\\ \\midrule
USC SAL        & %0.1f           & %0.1f       & -        &  \\\\
CMU NSH        & %0.1f           & %0.1f       & -        &  \\\\
Stanford Gates & %0.1f           & %0.1f       & -        &  \\\\
ACES           & %0.1f           & %0.1f       & -        & 41  \\\\
Intel Lab      & %0.1f           & %0.1f       & -        & 179 \\\\
MIT Killian    & %0.1f           & %0.1f       & -        & 190 \\\\ \\bottomrule
\\end{tabular}
""" % (
    *get_our_time("usc-sal"),
    *get_our_time("nsh_level_a"),
    *get_our_time("stanford-gates"),
    *get_our_time("aces"),
    *get_our_time("intel-lab"),
    *get_our_time("mit-killian")
)
print(table)