#!/usr/bin/env bash
# Free a TCP port by killing whatever is listening on it.
#
#   ./frontend/killport.sh            # frees the default port 8000
#   ./frontend/killport.sh 9000       # frees port 9000
#
# Useful when `python3 frontend/run.py` fails with:
#   [Errno 48] error while attempting to bind on address ('127.0.0.1', 8000):
#   address already in use
# — usually a previous uvicorn/reloader process that didn't shut down cleanly.

set -euo pipefail

PORT="${1:-8000}"

# PIDs listening on the port (LISTEN sockets only).
PIDS="$(lsof -nP -iTCP:"${PORT}" -sTCP:LISTEN -t 2>/dev/null || true)"

if [[ -z "${PIDS}" ]]; then
  echo "Port ${PORT} is free — nothing to kill."
  exit 0
fi

echo "Processes listening on port ${PORT}:"
lsof -nP -iTCP:"${PORT}" -sTCP:LISTEN 2>/dev/null || true

# Try a graceful SIGTERM first, then SIGKILL anything still holding the port.
echo "Sending SIGTERM to: ${PIDS//$'\n'/ }"
kill ${PIDS} 2>/dev/null || true
sleep 1

STILL="$(lsof -nP -iTCP:"${PORT}" -sTCP:LISTEN -t 2>/dev/null || true)"
if [[ -n "${STILL}" ]]; then
  echo "Still alive — sending SIGKILL to: ${STILL//$'\n'/ }"
  kill -9 ${STILL} 2>/dev/null || true
  sleep 1
fi

if [[ -z "$(lsof -nP -iTCP:"${PORT}" -sTCP:LISTEN -t 2>/dev/null || true)" ]]; then
  echo "Port ${PORT} is now free."
else
  echo "WARNING: port ${PORT} is still in use. You may lack permission to kill the owner." >&2
  exit 1
fi
