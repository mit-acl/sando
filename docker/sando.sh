#!/bin/bash

# SANDO Docker Entrypoint
# Usage via Makefile:
#   make run-demo SCENARIO=static_easy
#   make run-interactive
#   make run-interactive NUM_OBSTACLES=100

SANDO_WS="${SANDO_WS:-/root/sando_ws}"

source ~/.bashrc
source ${SANDO_WS}/install/setup.bash
source /usr/share/gazebo/setup.sh

# If USE_XPRA is set, start Xpra server for remote window forwarding (Mac workflow).
# RViz runs against a virtual display and is streamed to http://localhost:8080.
if [ "${USE_XPRA}" = "true" ]; then
    unset LIBGL_ALWAYS_INDIRECT
    export LIBGL_ALWAYS_SOFTWARE=1
    export GALLIUM_DRIVER=llvmpipe
    xpra start :100 \
        --bind-tcp=0.0.0.0:8080 \
        --html=on \
        --encoding=jpeg \
        --quality=85 \
        --speed=70 \
        --min-speed=50
    sleep 5
    export DISPLAY=:100
    echo "[SANDO] Xpra server running on port 8080"
    echo "[SANDO] Open in browser: http://localhost:8080"
fi

cd ${SANDO_WS}

MODE="${MODE:-demo}"
SCENARIO="${SCENARIO:-static_easy}"
SETUP_BASH="${SANDO_WS}/install/setup.bash"

if [ "$MODE" = "demo" ]; then
    case "$SCENARIO" in
        static_easy|static_medium|static_hard)
            TYPE="static"
            DIFFICULTY="${SCENARIO#static_}" ;;
        dynamic_easy|dynamic_medium|dynamic_hard)
            TYPE="dynamic"
            DIFFICULTY="${SCENARIO#dynamic_}" ;;
        unknown_dynamic_easy|unknown_dynamic_medium|unknown_dynamic_hard)
            TYPE="unknown_dynamic"
            DIFFICULTY="${SCENARIO#unknown_dynamic_}" ;;
        *)
            echo "Unknown scenario: $SCENARIO"
            echo "Available: static_easy, static_medium, static_hard,"
            echo "           dynamic_easy, dynamic_medium, dynamic_hard,"
            echo "           unknown_dynamic_easy, unknown_dynamic_medium, unknown_dynamic_hard"
            exit 1 ;;
    esac
    echo "[SANDO] Demo: $TYPE $DIFFICULTY"
    python3 src/sando/scripts/run_sim.py --mode "$TYPE" --difficulty "$DIFFICULTY" -s "$SETUP_BASH"

elif [ "$MODE" = "interactive" ]; then
    NUM_OBSTACLES="${NUM_OBSTACLES:-50}"
    DYNAMIC_RATIO="${DYNAMIC_RATIO:-0.65}"
    echo "[SANDO] Interactive mode: $NUM_OBSTACLES obstacles, ${DYNAMIC_RATIO} dynamic ratio"
    echo "[SANDO] Use RViz '2D Nav Goal' to send goals by clicking"
    python3 src/sando/scripts/run_sim.py \
        --mode interactive \
        --num-obstacles "$NUM_OBSTACLES" \
        --dynamic-ratio "$DYNAMIC_RATIO" \
        -s "$SETUP_BASH"

else
    echo "Unknown MODE: $MODE (expected 'demo' or 'interactive')"
    exit 1
fi
