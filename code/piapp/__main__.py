"""GPL-3.0-only. Launch the candidate Pi service: PYTHONPATH=code python -m piapp."""
import argparse
from dataclasses import replace
import math
from pathlib import Path
import signal
import sys
import threading

from .config import load


def main(argv=None):
    parser = argparse.ArgumentParser(description='CANSAT candidate Pi serial/camera/navigation service (host verified only)')
    parser.add_argument('--config', type=Path, required=True, help='TOML service configuration')
    parser.add_argument('--check-config', action='store_true', help='validate TOML and navigation options without hardware')
    parser.add_argument('--serial-replay', type=Path, help='paced raw STM32 byte recording instead of UART (bench only)')
    parser.add_argument('--duration', type=float, help='stop after this many seconds (bench only)')
    parser.add_argument('--no-poweroff', action='store_true', help='override allow_poweroff for bench use')
    args = parser.parse_args(argv)
    try:
        config = load(args.config)
        # Lazy even for --help, so package inspection does not construct maps or
        # import pyserial/Picamera2. Navigation validates its own options.
        from cansat.navigator import Navigator, NavigatorConfig
        nav_config = NavigatorConfig(**config.navigator)
        if args.check_config:
            print('Configuration valid (site file and hardware not opened).')
            return 0
        if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
            raise ValueError('--duration must be finite and positive')
        if config.site is None:
            raise ValueError('service.site is required; prepare a calibrated sitemap NPZ first')
        if args.no_poweroff or args.serial_replay is not None:
            config = replace(config, allow_poweroff=False)
        if args.serial_replay is not None and config.camera.kind == 'picamera2':
            raise ValueError('serial replay requires a synthetic or replay camera')
        from cansat import sitemap
        from .camera import create
        from .health import HealthMonitor
        from .service import Service
        from .storage import Writer
        from .transport import ReplaySerial, SerialPort
        site = sitemap.load(config.site)
        if (config.camera.width, config.camera.height) != (site.camera.width, site.camera.height):
            raise ValueError('camera dimensions differ from calibrated site camera')
        writer = Writer(config.data_dir, config.writer_capacity, config.sync_interval_s)
        # Closure is called only by Navigator on its owning worker. Read its
        # executing generation, rather than mislabelling late acquisition logs
        # with a generation received while that call was still in progress.
        service_ref = []

        def nav_log(record):
            generation = service_ref[0].worker.executing_generation if service_ref else 0
            writer.record('navigator', {**record, 'shell_generation': generation})

        navigator = Navigator(nav_config, site, log=nav_log)
        camera = create(config.camera)
        transport = (ReplaySerial(args.serial_replay, config.baud) if args.serial_replay is not None
                     else SerialPort(config.port, config.baud))
        health = HealthMonitor()
        stop = threading.Event()
        service = Service(config, transport, camera, navigator, writer, health=health)
        service_ref.append(service)
        previous = {}
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, lambda *_: stop.set())
        print(f'Session: {writer.session_dir}', flush=True)
        try:
            health.start()
            service.run(stop, args.duration)
        finally:
            health.stop()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
        if not service.shutdown_ok or service.fault or camera.error:
            print(f'Service fault: {service.fault or camera.error or writer.error or "unclean shutdown"}', file=sys.stderr)
            return 1
        return 0
    except (ValueError, TypeError, OSError, ImportError, KeyError) as error:
        print(f'piapp: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
