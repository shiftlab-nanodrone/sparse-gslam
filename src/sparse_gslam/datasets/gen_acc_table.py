def get_error_std(fname):
    return list(map(lambda x: float(x), open(fname).readlines()[1].split(", ")[:2]))

tb = """
\\begin{tabular}{@{}ccc@{}}
\\toprule
                                                 & \multicolumn{1}{c}{Ours}               & \multicolumn{1}{c}{Gmapping}          \\\\ \midrule
\multicolumn{1}{l}{Aces}                         & \multicolumn{1}{c}{4pt}                & \multicolumn{1}{c}{30pt} \\\\
\multicolumn{1}{l}{\quad Absolute translational} & \multicolumn{1}{c}{$%0.4f \pm %0.4f$}  & \multicolumn{1}{c}{$%0.4f \pm %0.4f$}   \\\\
\multicolumn{1}{l}{\quad Absolute rotational}    & \multicolumn{1}{c}{$%0.3f \pm %0.3f$}  & \multicolumn{1}{c}{$%0.3f \pm %0.3f$}   \\\\
\midrule
\multicolumn{1}{l}{Intel-Lab}                    & \multicolumn{1}{c}{13pt}               & \multicolumn{1}{c}{30pt} \\\\
\multicolumn{1}{l}{\quad Absolute translational} & \multicolumn{1}{c}{$%0.4f \pm %0.4f$}  & \multicolumn{1}{c}{$%0.4f \pm %0.4f$}   \\\\
\multicolumn{1}{l}{\quad Absolute rotational}    & \multicolumn{1}{c}{$%0.3f \pm %0.3f$}  & \multicolumn{1}{c}{$%0.3f \pm %0.3f$}   \\\\
\midrule
\multicolumn{1}{l}{MIT-Killian}                  & \multicolumn{1}{c}{11pt}               & \multirow{3}{*}{Fail to produce map} \\\\
\multicolumn{1}{l}{\quad Absolute translational} & \multicolumn{1}{c}{$%0.4f \pm %0.4f$}  &    \\\\
\multicolumn{1}{l}{\quad Absolute rotational}    & \multicolumn{1}{c}{$%0.3f \pm %0.3f$}  &    \\\\
\\bottomrule
\end{tabular}
""" % (
    *get_error_std("aces/aces-11_trans_error.log"),
    *get_error_std("aces/aces-gmapping-30_trans_error.log"),
    *get_error_std("aces/aces-11_rot_error.log"),
    *get_error_std("aces/aces-gmapping-30_rot_error.log"),
    
    *get_error_std("intel-lab/intel-lab-11_trans_error.log"),
    *get_error_std("intel-lab/intel-lab-gmapping-30_trans_error.log"),
    *get_error_std("intel-lab/intel-lab-11_rot_error.log"),
    *get_error_std("intel-lab/intel-lab-gmapping-30_rot_error.log"),

    *get_error_std("mit-killian/mit-killian-11_trans_error.log"),
    *get_error_std("mit-killian/mit-killian-11_rot_error.log")
)
print(tb)