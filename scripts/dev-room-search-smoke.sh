#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
port=$(python3 - <<'PY'
import socket
with socket.socket() as sock:
    sock.bind(('127.0.0.1', 0))
    print(sock.getsockname()[1])
PY
)
log=$(mktemp /tmp/room-search-http.XXXXXX.log)
"$root/scripts/dev-room-search.sh" --port "$port" >"$log" 2>&1 &
pid=$!
cleanup() {
    kill "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    rm -f "$log"
}
trap cleanup EXIT

ready=false
for _ in {1..30}; do
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "room-search smoke server exited before readiness" >&2
        cat "$log" >&2
        exit 1
    fi
    if curl -fsS "http://127.0.0.1:$port/api/health" >/dev/null 2>&1; then
        ready=true
        break
    fi
    sleep .1
done
if [[ "$ready" != true ]]; then
    echo "room-search smoke server did not become ready" >&2
    cat "$log" >&2
    exit 1
fi

call() {
    curl -fsS -X POST "http://127.0.0.1:$port/api/$1" \
        ${2:+-d "$2"} -H 'Content-Type: application/json'
}
curl -s -o /tmp/room-search-denied.json -w '%{http_code}' \
    -X POST "http://127.0.0.1:$port/api/confirm_intent" | grep -qx 409
call load >/dev/null
call proposal '{"text":"去书房找书桌"}' >/dev/null
call confirm_intent >/dev/null
call scan_goal >/dev/null
call start_scan >/dev/null
call finish_scan >/dev/null
call analyze >/dev/null
call preview >/dev/null
call confirm_dock >/dev/null
call dock_goal | grep -q 'ARRIVED'
call cancel | grep -q 'CANCELED'
echo 'HTTP consumer smoke passed: isolated server, 409 gate, ARRIVED, cancel'
