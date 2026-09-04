#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd); port=18088
"$root/scripts/dev-room-search.sh" --port "$port" >/tmp/room-search-http.log 2>&1 & pid=$!
trap 'kill $pid 2>/dev/null || true' EXIT
for x in {1..20}; do curl -fsS "http://127.0.0.1:$port/api/health" >/dev/null && break; sleep .1; done
call(){ curl -fsS -X POST "http://127.0.0.1:$port/api/$1" ${2:+-d "$2"} -H 'Content-Type: application/json'; }
curl -s -o /tmp/room-search-denied.json -w '%{http_code}' -X POST "http://127.0.0.1:$port/api/confirm_intent" | grep -qx 409
call load >/dev/null; call proposal '{"text":"去书房找书桌"}' >/dev/null; call confirm_intent >/dev/null; call scan_goal >/dev/null; call at_scan >/dev/null; call start_scan >/dev/null; call finish_scan >/dev/null; call analyze >/dev/null; call preview >/dev/null; call confirm_dock >/dev/null; call dock_goal >/dev/null; call arrived | grep -q 'ARRIVED'
call cancel | grep -q 'CANCELED'
echo 'HTTP consumer smoke passed: 409 gate, ARRIVED, cancel'
