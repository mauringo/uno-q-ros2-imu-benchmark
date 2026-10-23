"""App Lab entry point. Keep this process idle to avoid competing Bridge calls."""
import time

print("UNO Q ROS 2 speed benchmark runs automatically in the ros2 Brick.", flush=True)
print("No sensor is required. No console RPC traffic is added during testing.", flush=True)
print('Board shell: find "$HOME/ArduinoApps" -name UNO_Q_SPEED_APP.marker '
      '-execdir docker compose -f .cache/app-compose.yaml logs -f ros2 \\; -quit', flush=True)
print("Results: results/<UTC run ID>/summary.json and stages.csv in the deployed app.", flush=True)
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
