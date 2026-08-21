# ORB_SLAM3_Modern

A modern-C++ restructuring of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) (V1.0),
built and validated for **arm64 / Apple Silicon Docker**. The goal is a maintainable,
readable, memory-safe codebase that treats the original's accuracy as a **floor, not a
target** — every mode is required to match or beat the upstream baseline.

**Upstream authors:** Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
José M. M. Montiel, Juan D. Tardós (University of Zaragoza).
This repository is a GPLv3 derivative; all credit for the SLAM system itself belongs to them.

## What changed vs. upstream

| Area | Upstream V1.0 | This repository |
|---|---|---|
| Toolchain | C++11, Ubuntu 18/20 era | **C++20**, GCC 13, Ubuntu 24.04 container |
| Layout | flat `include/`+`src/` (60+ files) | 12 module directories (`core/ camera/ features/ map/ backend/ geometry/ tracking/ mapping/ recognition/ closing/ io/ viz/`) |
| Memory | raw `new`/`delete`, known leaks | **`shared_ptr` ownership graph** — zero manual KeyFrame/MapPoint deletes, `Map::clear()` genuinely frees, ASan-clean full runs (see `docs/OWNERSHIP.md`) |
| Concurrency | polling loops, data races | event-driven CVs (queue pickup ~1.8 ms → ~22 µs), atomics with documented contracts, ordered shutdown join chain, TSAN-audited (race ledger fully retired) |
| g2o | vendored ORB-SLAM fork | **upstream g2o `20241228_git` submodule**, fork behavior restored in wrappers (`OrbLevenberg`, vendored-LDLT solver header); `Optimizer.cpp` (5,846 lines) decomposed into 6 function-family TUs with graph-builder helpers — every optimizer reads as `vertices → edges → solve → recover` |
| DBoW2 | vendored fork | **upstream dorian3d submodule** + `OrbVocabulary` wrapper (text loaders, binary cache: vocabulary load 2.1 s → 0.08 s warm) |
| Eigen / Pangolin | system packages | **Eigen 5.0.1 + Pangolin v0.9.6 submodules**, single-Eigen pin across all three consumers |
| Dead weight | — | commented-out code fully adjudicated (197 findings: 1 implemented, 193 removed, 4 kept with rationale), debug scaffolding retired, warnings 450 → 0 in `src/include/tests` |

Deliberate divergences from upstream (bug fixes, solver-robustness restorations, and their
evidence) are cataloged in [`docs/DIVERGENCES.md`](docs/DIVERGENCES.md).

## Accuracy vs. upstream (final validation, 2026-08)

Median ATE RMSE, this repo vs. upstream binaries on the same machine (EuRoC MH01, KITTI 00):

| Mode | This repo | Upstream | Δ |
|---|---|---|---|
| EuRoC mono-inertial | 0.0534 | 0.0773 | **−31 %** |
| EuRoC stereo-inertial | 0.0379 | 0.0394 | **−3.8 %** |
| KITTI 00 stereo (loop closures fire) | 1.180 | 1.207 | **−2.3 %** |
| EuRoC stereo | parity (interleaved A/B: −0.9 %) | | |
| EuRoC mono | within noise (interleaved A/B: +13.6 %, no pairwise direction) | | |

Full methodology and the machine-state control experiment: [`docs/REFACTOR_PLAN.md`](docs/REFACTOR_PLAN.md).

## Build (Docker, arm64)

```bash
git clone --recurse-submodules https://github.com/Hyeonvidia/ORB_SLAM3_Modern.git
cd ORB_SLAM3_Modern
docker compose build
docker compose run --rm dev docker/scripts/container_build.sh
```

The build lands in the `/build` named volume; the ORB vocabulary is extracted (and binary-cached
on first load) automatically.

## Run

```bash
# headless, results under ./results/<mode>_<seq>_<timestamp>/
docker compose run --rm -e HEADLESS=1 dev docker/scripts/run_slam.sh euroc_stereo MH01
docker compose run --rm -e HEADLESS=1 dev docker/scripts/run_slam.sh euroc_mono_inertial MH01
docker compose run --rm -e HEADLESS=1 dev docker/scripts/run_slam.sh kitti_stereo 00
```

Modes: `euroc_mono | euroc_stereo | euroc_mono_inertial | euroc_stereo_inertial | kitti_stereo`.
Datasets are mounted from `../Datasets` (see `docker-compose.yml`). EuRoC downloads:
the old ASL URLs are dead — use the
[ETH Research Collection DOI](https://doi.org/10.3929/ethz-a-010702001). For the GUI viewer
on macOS, run without `HEADLESS=1` under XQuartz.

## Verification harness

- `benchmark/scripts/smoke_gate.sh` — one stereo MH01 run against a sanity ATE bound (~4 min)
- `benchmark/scripts/bit_gate.sh` — 9 feature-layer hashes, bit-identical tripwire for numeric drift
- `tests/backend_equiv/` — two-binary optimizer equivalence harness (frozen pre-migration twin
  vs. modern backend, per-record SHA comparison)
- `tests/` unit binaries (threading contracts, vocabulary round-trip, solver-fix fixtures)
- `benchmark/golden/` — upstream baseline ATE distributions used as the regression floor

## Documentation map

- [`docs/REFACTOR_PLAN.md`](docs/REFACTOR_PLAN.md) — the R-series plan with per-step completion evidence (governing document)
- [`docs/DIVERGENCES.md`](docs/DIVERGENCES.md) — every deliberate behavioral divergence, with rationale
- [`docs/OWNERSHIP.md`](docs/OWNERSHIP.md) — smart-pointer ownership tables, tombstone/lock contracts
- [`docs/IMU_CONTRACT.md`](docs/IMU_CONTRACT.md) — IMU pipeline invariants
- [`docs/phase_reports/`](docs/phase_reports/) — the earlier P-series (P1–P11) engineering reports

## License and citation

GPLv3, as a derivative of ORB-SLAM3 (see [LICENSE](LICENSE) and
[Dependencies.md](Dependencies.md)). If you use this in academic work, cite the original:

> Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel and Juan D. Tardós,
> **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**,
> *IEEE Transactions on Robotics 37(6):1874-1890, Dec. 2021*. [PDF](https://arxiv.org/abs/2007.11898)

Further upstream publications (ORB-SLAM2, ORBSLAM-Atlas, IMU initialization, DBoW2) are listed in
the [upstream README](https://github.com/UZ-SLAMLab/ORB_SLAM3#related-publications).
