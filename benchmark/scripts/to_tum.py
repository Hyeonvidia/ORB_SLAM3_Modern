#!/usr/bin/env python3
"""Convert ORB-SLAM3 trajectories and EuRoC ground truth to TUM format (seconds).

Input kinds:
  traj       ORB-SLAM3 EuRoC trajectory: "t_ns x y z qx qy qz qw" (space-sep)
  euroc_csv  EuRoC-style CSV: "t_ns,px,py,pz,qw,qx,qy,qz[,...]"
             (covers both evaluation/Ground_truth/*_GT.txt and
              mav0/state_groundtruth_estimate0/data.csv)

Output: TUM "t_s x y z qx qy qz qw"
"""
import sys


def main() -> int:
    if len(sys.argv) != 4 or sys.argv[1] not in ("traj", "euroc_csv"):
        print(f"usage: {sys.argv[0]} traj|euroc_csv <in> <out>", file=sys.stderr)
        return 1
    kind, src, dst = sys.argv[1], sys.argv[2], sys.argv[3]
    n = 0
    with open(src) as f, open(dst, "w") as out:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            if kind == "traj":
                v = ln.split()
                t = float(v[0]) / 1e9
                x, y, z, qx, qy, qz, qw = (float(a) for a in v[1:8])
            else:
                v = [a for a in ln.split(",") if a != ""]
                t = float(v[0]) / 1e9
                px, py, pz = (float(a) for a in v[1:4])
                qw, qx, qy, qz = (float(a) for a in v[4:8])
                x, y, z = px, py, pz
            out.write(f"{t:.9f} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
            n += 1
    print(f"{dst}: {n} poses")
    return 0


if __name__ == "__main__":
    sys.exit(main())
