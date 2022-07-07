from spatialmath import SE2
import sys
import os

def read_as_dict(f):
    r_lines = f.readlines()
    r_lines = [line.split(" ") for line in r_lines if line.startswith("FLASER")]
    t_dict = dict()
    for line in r_lines:
        time = round(float(line[-3]), 6)
        t_dict[time] = SE2(
            float(line[-6]), float(line[-5]), float(line[-4])
        )
    return t_dict

dataset, in_f = os.path.split(sys.argv[1])
with open(sys.argv[1], "r") as f:
    r_dict = read_as_dict(f)

with open(os.path.join(dataset, f"{dataset}.log"), "r") as f:
    l_dict = read_as_dict(f)

for key in r_dict:
    assert key in l_dict

r_dict = list(r_dict.items())
r_dict.sort(key=lambda x:x[0])
l_dict = list(l_dict.items())
l_dict.sort(key=lambda x:x[0])

r_time, r_se2 = r_dict[0]
new_results = [(r_time, r_se2.xyt())]
l_idx = 0
r_idx = 0
while r_dict[r_idx][0] != l_dict[l_idx][0]:
    l_idx += 1
print(l_idx, r_idx)
last_raw_odom: SE2 = l_dict[l_idx][1]
r_idx += 1
l_idx += 1
r_time, r_se2 = r_dict[r_idx]
while True:
    while r_time != l_dict[l_idx][0]:
        l_time, raw_odom = l_dict[l_idx]
        new_results.append((
            l_time, (r_dict[r_idx - 1][1] * last_raw_odom.inv() * raw_odom).xyt()
        ))
        l_idx += 1
    last_raw_odom = l_dict[l_idx][1]
    new_results.append((r_time, r_se2.xyt()))
    r_idx += 1
    l_idx += 1
    if r_idx >= len(r_dict):
        break
    r_time, r_se2 = r_dict[r_idx]

with open(os.path.join(dataset, f"{dataset}.result"), "w") as f:
    for row in new_results:
        time, (x, y, theta) = row
        f.write(
            f"FLASER 0 {x} {y} {theta} {x} {y} {theta} {time} myhost {time}\n"
        )

# print(r_dict.keys())
# new_lines = []
# for line in lines:
#     line = line.split(" ")
#     line = line[:2] + line[2:5] + line[2:5] + line[5:]
#     new_lines.append(" ".join(line))
# print(new_lines[0])
# with open("intel-lab.result", "w") as f:
#     f.writelines(new_lines)