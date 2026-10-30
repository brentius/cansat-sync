#!/bin/sh
# GPL-3.0-only. Explicit IMX296 external-trigger provisioning at every boot.
set -eu
parameter=/sys/module/imx296/parameters/trigger_mode
# The unit is ordered only after local-fs; the imx296 module may still be loading.
waited=0
while [ ! -w "$parameter" ]; do
    if [ "$waited" -ge 30 ]; then
        echo "IMX296 trigger parameter unavailable after ${waited} s: $parameter" >&2
        exit 1
    fi
    sleep 1
    waited=$((waited + 1))
done
printf '1\n' > "$parameter"
test "$(cat "$parameter")" = 1
