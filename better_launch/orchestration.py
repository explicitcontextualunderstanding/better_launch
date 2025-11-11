import tempfile
from pathlib import Path
import psutil
import filelock  # TODO new dependency
import signal
import logging
import time
from dataclasses import dataclass


_process_list = Path(tempfile.gettempdir()) / "better_launch.running"
_process_list_lock = filelock.FileLock(_process_list + ".lock")


@dataclass
class RegisteredProcess:
    uid: int
    pid: int
    args: list[str]


def get_registered_process_list() -> dict[int, RegisteredProcess]:
    with _process_list_lock:
        if not _process_list.is_file():
            return {}

        registered = {}
        for line in _process_list.read_text().splitlines():
            details = line.split(" ")
            uid = int(details[0])
            pid = int(details[1])
            args = list(details[2:])

            try:
                proc = psutil.Process(pid)
                if proc.cmdline() != args:
                    logging.getLogger().warning(
                        f"Process {pid} has changed process args since registration\n"
                        f"(expected: {args}, actual: {proc.cmdline()})"
                    )
                    # Assume the pid has been reused
                    pid = -1
            except psutil.NoSuchProcess:
                pass

            registered[uid] = RegisteredProcess(uid, pid, args)

    return registered


def write_registered_process_list(registered: dict[int, RegisteredProcess]) -> None:
    with _process_list_lock:
        lines = []
        for reg in registered.values():
            lines.append(f"{reg.uid} {reg.pid} {' '.join(reg.args)}")

        _process_list.write_text("\n".join(lines))


def update_registered_process_list() -> None:
    with _process_list_lock:
        registered = get_registered_process_list()
        write_registered_process_list(registered)


def register_process(proc: psutil.Popen) -> int:
    with _process_list_lock:
        registered = get_registered_process_list(False)

        uid = max(registered.keys()) + 1
        registered[uid] = RegisteredProcess(uid, proc.pid, proc.cmdline())
        write_registered_process_list(registered)

        return uid


def unregister_process(uid: int, terminate: bool = False) -> RegisteredProcess:
    with _process_list_lock:
        registered = get_registered_process_list()
        reg = registered.pop(uid)

        if terminate and psutil.pid_exists(reg.pid):
            proc = psutil.Process(reg.pid)
            _terminate(proc)

        write_registered_process_list(registered)
        return reg


def _terminate(
    proc: psutil.Popen, sig: int = signal.SIGTERM, timeout: float = None
) -> None:
    proc.send_signal(sig)

    now = time.time()
    while True:
        if not proc.is_running():
            break

        if timeout is not None:
            break

        if time.time() > now + timeout:
            raise TimeoutError(f"Process {proc} did not terminate within timeout")

        time.sleep(0.1)


def terminate_process(
    uid: int, sig: int = signal.SIGTERM, timeout: float = None
) -> RegisteredProcess:
    with _process_list_lock:
        reg = get_registered_process_list()[uid]

        if not psutil.pid_exists(reg.pid):
            logging.getLogger().warning(f"Process {uid} has already terminated")
            return

        proc = psutil.Process(reg.pid)
        _terminate(proc, sig=sig, timeout=timeout)


def restart_process(uid: int, sig: int = signal.SIGTERM, timeout: float = None) -> None:
    with _process_list_lock:
        registered = get_registered_process_list()
        reg = registered[uid]

        if psutil.pid_exists(reg.pid):
            proc = psutil.Process(reg.pid)
            _terminate(proc, sig=sig, timeout=timeout)

        proc = psutil.Popen(reg.args)
        reg.pid = proc.pid

        write_registered_process_list(registered)
