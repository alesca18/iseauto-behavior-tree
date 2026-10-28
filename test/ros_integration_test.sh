#!/usr/bin/env bash
# Test di integrazione ROS 2: brain_node (C++) e metadrive_bridge.py comunicano via
# /closest_obstacle e /cmd_vel. Passa se non ci sono collisioni e se ogni arresto
# avviene fra 4.8 e 5.2 m dall'ostacolo (Capitolo 5).
# Uso (workspace gia' compilato e sorgente): test/ros_integration_test.sh [episodi]
set -u
EPISODES=${1:-10}
LOG=$(mktemp)

ros2 run iseauto_bt brain_node > brain.log 2>&1 &
BRAIN_PID=$!
sleep 2

timeout 900 ros2 run iseauto_bt metadrive_bridge.py --episodes "$EPISODES" --headless 2>&1 | tee "$LOG"
kill "$BRAIN_PID" 2>/dev/null
wait "$BRAIN_PID" 2>/dev/null

echo "--- brain_node ---"
grep -E "Started|WCET|Telemetria" brain.log | tail -5

python3 - "$LOG" "$EPISODES" <<'EOF'
import re, sys
text = open(sys.argv[1]).read()
episodes = int(sys.argv[2])
done = re.findall(r"Test (\d+)/\d+: (.*)", text)
stops = [float(m) for m in re.findall(r"arresto a ([\d.]+) m", text)]
crashes = sum("COLLISIONE" in esito for _, esito in done)
errors = []
if len(done) != episodes:
    errors.append(f"completati {len(done)}/{episodes} test")
if crashes:
    errors.append(f"{crashes} collisioni")
if not stops:
    errors.append("nessun arresto davanti a un ostacolo")
bad = [d for d in stops if not 4.8 <= d <= 5.2]
if bad:
    errors.append(f"arresti fuori da 4.8-5.2 m: {bad}")
print(f"Test: {len(done)}  collisioni: {crashes}  arresti: {stops}")
if errors:
    print("FALLITO:", "; ".join(errors))
    sys.exit(1)
print("OK")
EOF
