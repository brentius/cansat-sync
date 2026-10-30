# Candidate Pi deployment

GPL-3.0-only. This package is host-verifiable candidate software. No Pi, camera,
UART, storage-power-cut or shutdown hardware tests have been performed.

## Provision and run

Use a headless Raspberry Pi OS image with Python **3.11 or newer**, NumPy,
`python3-picamera2`, `python3-serial`, and the matching libcamera/kernel packages
already provisioned. Record their versions for each image. No dependencies were
installed on the development laptop. Keep `code/cansat` and `code/piapp` together
under `/opt/cansat/code`, and install `code/deploy/pi/enable-trigger.sh` at
`/opt/cansat/code/deploy/pi/enable-trigger.sh` (the unit's `ExecStartPre`; it
waits up to 30 s for the imx296 module's `trigger_mode` parameter). A virtualenv must expose OS Picamera2/libcamera packages;
the supplied unit deliberately uses `/usr/bin/python3`.

1. Merge `config.txt.fragment` into `/boot/firmware/config.txt` and apply
   `cmdline.txt.notes` to the existing single-line command line. Disable
   `hciuart` and `serial-getty@ttyAMA0`. These changes free PL011 for the STM32.
   Reboot and verify `/dev/ttyAMA0` is the intended UART. GPIO15's pull-up is a
   boot setting; verify idle voltage with the actual buffer. See official
   [UART configuration](https://www.raspberrypi.com/documentation/computers/configuration.html)
   and [GPIO boot settings](https://github.com/raspberrypi/documentation/blob/master/documentation/asciidoc/computers/config_txt/gpio.adoc).
2. Provision and mount the writable data partition at `/data`. Put a calibrated
   sitemap bundle at `/opt/cansat/site.npz`; synthetic `sitemap.from_farmland`
   products are for bench use only. Camera dimensions must match the bundle.
   Distortion correction belongs to Navigator: the shell preserves raw pixels.
3. Install `flight.toml` and `camera-timeout.yaml` in `/etc/cansat`, and
   `cansat-pi.service` in `/etc/systemd/system`. The unit runs as root to access
   the trigger parameter and invoke `systemctl poweroff`; provision a more
   restrictive device/polkit policy before changing its user. Do not enable
   unreviewed configs: `allow_poweroff=true` makes valid STM32 SHUTDOWN halt OS.
4. Validate with `PYTHONPATH=/opt/cansat/code /usr/bin/python3 -m piapp
   --config /etc/cansat/flight.toml --check-config` (one shell line), inspect
   `systemd-analyze verify`, then enable/start `cansat-pi.service` after bench
   wiring and provisioning checks. `--check-config` validates syntax/options;
   it does not open the map or devices.

## Camera trigger and timing

Power on the complete CanSat before loading. Linux and the flight service must
already be running, with stable PiStatus heartbeats and map/camera/storage ready,
before sealing the rocket. Leave the Pi powered during the full pad hold and
launch preparation. The flight MCU default keeps power on even if UART traffic
is absent; a missing heartbeat invalidates secondary data, rather than rebooting
Linux. `capture_on_pad=false` only delays image capture/processing; it does not
power off or suspend the Pi. The supplied 2025-26 guidelines require three-hour
switched-on capability and operation during the 5–10 minute loaded launch wait.
An MCU reset is a fault that can interrupt the Pi rail; repeat the boot/readiness
check afterward. Physical continuity across MCU reset is not guaranteed by this
software and needs hardware validation.

IMX296 XTR is a **1.8 V input**, active low. Board variants with Q2 fitted may
require R11 removal; inspect the board before modifying it. The boot helper sets
`/sys/module/imx296/parameters/trigger_mode` to `1`. The sensor's exposure is
the low pulse plus 14.26 microseconds. Fixed exposure/gain controls avoid AE/AWB
settling. The explicit `imx296,always-on` overlay is used here for controlled
provisioning. See Raspberry Pi's
[external-trigger instructions](https://github.com/raspberrypi/documentation/blob/master/documentation/asciidoc/accessories/camera/external_trigger.adoc).

The callback copies the full Y plane of YUV420, crops stride padding, and records
receive monotonic time before copying. It does not rectify or bin images.
[Picamera2 source](https://github.com/raspberrypi/picamera2/blob/main/picamera2/picamera2.py)
and [request buffer layout](https://github.com/raspberrypi/picamera2/blob/main/picamera2/request.py)
were consulted. The assumed 20 ms trigger-to-callback latency **must be measured**;
the existing FrameStamper maps to exposure-centre STM32 time. Callback time is
not a measured hardware exposure timestamp. Reject frames with no recent trigger,
duplicate timestamps, stale generations, or no stamp within the wait window.

The camera may wait on the pad for hours. `camera-timeout.yaml` selects a
four-hour timeout [assumed] and suppresses startup frame drops, while leaving
STM32 triggers disabled until flight/PREPARE. Raspberry Pi's
[VC4 pipeline template](https://raw.githubusercontent.com/raspberrypi/libcamera/main/src/libcamera/pipeline/rpi/vc4/data/rpi_apps.yaml)
defines the keys: **zero requests the computed timeout, not unlimited**. Verify
the installed library accepts this file and survives the full pad duration.
The template is VC4-specific, not a Pi 5 configuration.

## Runtime and storage policy

The UART thread sends PiStatus at 2 Hz, cached Nav at 5 Hz, and TrigCfg on change
and at 1 Hz. Only the navigation worker touches Navigator. Its sensor FIFO holds
1500 messages [assumed, roughly 12 seconds at nominal rates]; its frame slot
keeps the newest frame. Overflow counts discarded messages and forces navigation
restart. Healthy-capacity delivery preserves every sensor's arrival order. A
finite queue cannot preserve an unlimited acquisition stall. Serial heartbeat
does not wait for acquisition or SD writes; CPython scheduling and target CPU
contention still require measurement. Nav cache and STM32 solution age are both
checked before transmission.

Each process launch creates a unique session directory. `raw_rx.bin` and
`raw_tx.bin` retain accepted writer chunks; `decoded.jsonl` records message type,
direction, sequence on RX, generation and receive time. `navigator.jsonl` keeps
Navigator records, including its comparison branches. `events.jsonl` records
faults/restarts/commands; `frames.jsonl` maps generation, trigger counter, float
exposure-centre tick, callback time and NPY filename. Stored greyscale pixels are
unchanged. Files never overwrite previous sessions or raw measurements. There
is no automatic deletion or disk rotation; provision enough capacity and archive
between flights. At five full-resolution uint8 frames/s, raw image data alone
is about 7.9 MB/s [calculation]; three hours of pad capture would be about 85 GB.

The writer has 32 slots [assumed], shared between records and images. If storage
lags it drops new work, counts losses, and never stalls UART. Thus raw logs may
have gaps under overload. IO errors mark storage unhealthy and suppress a clean
shutdown acknowledgement. Frames use exclusive temporary files and atomic rename;
metadata and directory entries are synced periodically (1 s default), each image
is synced before indexing, and orderly shutdown drains/syncs. Power loss can leave
unindexed frames, partial final JSON rows, or `.part` images: retain them for
recovery. fsync cannot guarantee the SD card's power-loss behaviour.

SHUTDOWN is idempotent by command sequence/type. It acknowledges accepted, stops
camera and navigation, drains/syncs logs, acknowledges done, drains UART, then
calls an injectable poweroff hook. Before the final ACK starts, timeout or storage
failure withdraws unsent done and caches refusal. A busy Python acquisition cannot
be forcibly cancelled: if cleanup exceeds the budget, done is withheld. Once a
clean write accepts the first final-ACK byte, the service finishes/drains the frame
and retries halt even after that local deadline. STM32 still owns the independent
hard power-removal deadline. An ambiguous write exception quarantines subsequent
TX and withholds local halt; bytes already accepted cannot be recalled, including
a whole DONE frame accepted before an exception. Actual UART acceptance/drain and
OS halt need bench verification. SIGTERM stops cleanly without calling poweroff.
Heartbeat/control may continue during cleanup. Final post-sync wire acknowledgements
cannot be present in the already closed SD log. Inspect journal and STM32 logs for
that tail. See the [implementation report](../../../wiki/flight_software.md).

For a read-only root, provision all packages/map/config/unit beforehand; enable
the OS overlay/read-only-root mechanism, place runtime state in tmpfs, and keep
`/data` on a separately writable filesystem. The unit asserts `/data` is a mount
point so a missing partition cannot silently fill the root. Keep journal volatile
or on the data partition. Test dirty-power recovery, mount/fsck policy, and the
STM32 timeout path on the actual card. These are deployment notes, not an applied
or qualified OS image.

## Host use and verification

Run `.venv/Scripts/python.exe -m pytest code/piapp/tests` from the repository.
With `code/` on PYTHONPATH, `python -m piapp --help` needs no hardware packages.
For CLI bench use, point a TOML at a synthetic site and camera and run
`python -m piapp --config bench.toml --serial-replay raw_rx.bin --duration 2
--no-poweroff` (one shell line). Raw replay is paced by UART byte rate; it does
not reproduce original message/camera timing. `ReplayCamera` rebases the relative
`local_s` times in a frame index; a coordinated timestamp-faithful flight replay
belongs to the navigation replay harness. Replay never imports pyserial and
forces actual OS poweroff off. There are no real-image or target timing claims.
