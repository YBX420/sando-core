#!/usr/bin/env bash
# metadrone.sh — one launcher for the whole scenario workbench (no more hand-typing env recipes).
#
#   ./metadrone.sh editor  [--seed 3] [--load scenarios/x.json]     interactive 3D editor (Fusion360 mouse)
#   ./metadrone.sh fly     scenarios/x.json [--live] [--mp4] [...]  fly with the REAL safety layer (3D)
#   ./metadrone.sh preview scenarios/x.json                         headless 3D PNG frames
#   ./metadrone.sh eval    scenarios/x.json [--variants N] [--resample N] [PERCEPT=realistic 前置亦可]
#   ./metadrone.sh ab      scenarios/bench/street_rush_s1.json   ours/native(观众机位,快)
#   ./metadrone.sh abr     scenarios/bench/x.json --w 1280 --h 800   旧全传感器渲染器 AB(FPV/追拍/KF 叠加)
#   ./metadrone.sh gen     --seed 0 --peds 5 --crossers 3 --vehicles 3   街景人车生成器
#   ./metadrone.sh ui                                               open the web workbench (browser)
#   ./metadrone.sh test                                             all selftests + ctest
#
# Env overrides: METAURBAN_ROOT, SANDO_DISPLAY (default: current $DISPLAY, else :1), PERCEPT=gt|realistic
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$(dirname "$HERE")")"                              # .../sando_py
export METAURBAN_ROOT="${METAURBAN_ROOT:-$ROOT/metaurban}"
[ -d "$METAURBAN_ROOT/metaurban" ] || METAURBAN_ROOT=/media/boxuan/Data2/projects/metaurban
MU_PY="$HOME/miniconda3/envs/metaurban/bin/python"
SANDO_STDCXX="$HOME/miniconda3/envs/sando/lib/libstdc++.so.6"
DISP="${SANDO_DISPLAY:-${DISPLAY:-:1}}"

cmd="${1:-help}"; shift || true
cd "$HERE"

case "$cmd" in
  editor)   # 3D interactive editor: needs GL window; no ego .so -> no LD_PRELOAD
    exec env DISPLAY="$DISP" PYTHONPATH="$METAURBAN_ROOT" \
      "$MU_PY" scenario_designer3d.py "$@"
    ;;
  fly)      # real safety layer + 3D render: needs GL + ego_capi -> LD_PRELOAD required
    exec env DISPLAY="$DISP" PYTHONPATH="$METAURBAN_ROOT" LD_PRELOAD="$SANDO_STDCXX" \
      "$MU_PY" run_scenario_3d.py "$@"
    ;;
  preview)  # headless 3D frames
    exec env DISPLAY="$DISP" PYTHONPATH="$METAURBAN_ROOT" \
      "$MU_PY" scenario_designer3d.py --preview "$@"
    ;;
  eval)     # headless evaluation (sando-side python; MetaUrban not needed)
    exec python3 run_scenario.py "$@"
    ;;
  gen)      # procedural street-life generator (realistic paths + spacing control)
    exec env DISPLAY="$DISP" PYTHONPATH="$METAURBAN_ROOT" \
      "$MU_PY" populate.py "$@"
    ;;
  ab)       # A/B: fly ours THEN native on the same scenario -> two mp4s side by side material
    scnf="$1"; shift || true
    env DISPLAY="$DISP" PYTHONPATH="$METAURBAN_ROOT" LD_PRELOAD="$SANDO_STDCXX" \
      "$MU_PY" run_scenario_3d.py "$scnf" --mode ours --mp4 "$@"
    env DISPLAY="$DISP" PYTHONPATH="$METAURBAN_ROOT" LD_PRELOAD="$SANDO_STDCXX" \
      "$MU_PY" run_scenario_3d.py "$scnf" --mode native --mp4 "$@"
    n=$(basename "$scnf" .json)
    "$MU_PY" stitch_ab.py "out/scenario_videos/${n}_ours.mp4" "OURS (certified)" \
      "out/scenario_videos/${n}_native.mp4" "NATIVE EGO" "out/scenario_videos/${n}_AB.mp4" 2>/dev/null \
      || echo "[ab] stitch_ab.py 接口不匹配 -> 两个 mp4 已在 out/scenario_videos/,手动并排"
    ;;
  abr)      # A/B on the FULL-SENSOR legacy renderer (cone+D435i-capable, dual FPV/chase views, KF overlays):
    scnf="$1"; shift || true                     #   ./metadrone.sh abr scenarios/bench/x.json --w 1280 --h 800
    n=$(basename "$scnf" .json)
    env DISPLAY="$DISP" PYTHONPATH="$METAURBAN_ROOT" LD_PRELOAD="$SANDO_STDCXX" \
      "$MU_PY" render_3d_video.py --seed 3 --scenario "$scnf" --no_crowd --maneuver --mp4 \
        --d435i --fov_range 10 "$@"
    cp out/drone_3d.mp4 "out/scenario_videos/${n}_r_ours.mp4"
    env DISPLAY="$DISP" PYTHONPATH="$METAURBAN_ROOT" LD_PRELOAD="$SANDO_STDCXX" \
      "$MU_PY" render_3d_video.py --seed 3 --scenario "$scnf" --no_crowd --ego --mp4 \
        --d435i --fov_range 10 "$@"
    cp out/drone_3d.mp4 "out/scenario_videos/${n}_r_native.mp4"
    "$MU_PY" stitch_ab.py "out/scenario_videos/${n}_r_ours.mp4" "OURS (certified)" \
      "out/scenario_videos/${n}_r_native.mp4" "NATIVE EGO" "out/scenario_videos/${n}_rAB.mp4"
    ;;
  ui)
    exec xdg-open "$HERE/scenario_workbench.html"
    ;;
  test)
    python3 perception.py
    python3 kf_tracker.py | grep PASS
    python3 scenario_designer3d.py --selftest
    env SDL_VIDEODRIVER=dummy "$MU_PY" scenario_designer.py --selftest 2>&1 | tail -1
    for f in scenarios/*.json; do python3 -c "import scenario_lib as S; S.to_movers_raw(S.load('$f'))" \
      && echo "OK $f" || echo "FAIL $f"; done
    python3 verify_spacing.py scenarios/bench | tail -2
    ( cd ../cpp/build && ctest | tail -1 )
    # .so freshness guard (the audit's staleness trap):
    for pair in "../ego/capi/ego_capi.so ../ego/src ../ego/capi/ego_capi.cpp ../cpp/include/sando_cpp/bernstein_cert.hpp" \
                "../cpp/capi/cert_capi.so ../cpp/capi/cert_capi.cpp ../cpp/include/sando_cpp/bernstein_cert.hpp"; do
      set -- $pair; so=$1; shift
      if [ -n "$(find "$@" -newer "$so" 2>/dev/null | head -1)" ]; then
        echo "STALE $so -- rebuild it (see CLAUDE.md capi recipe)"; else echo "FRESH $so"; fi
    done
    ;;
  *)
    grep '^#   ' "$0" | sed 's/^#   //'
    ;;
esac
