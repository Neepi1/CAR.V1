#!/usr/bin/env python3
"""Read /proc only; no DDS participant or API calls. Emits JSON to stdout."""
import json
from pathlib import Path
import time


def snapshot():
    processes = []
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        try:
            paths = sorted({line.split()[-1] for line in
                            (directory / 'maps').read_text().splitlines()
                            if '/libfastrtps.so' in line and '(deleted)' not in line})
            deleted = any('/libfastrtps.so' in line and '(deleted)' in line
                          for line in (directory / 'maps').read_text().splitlines())
            if not paths and not deleted:
                continue
            # Do not expose command arguments: API credentials may be present.
            processes.append({
                'pid': int(directory.name),
                'comm': (directory / 'comm').read_text().strip(),
                'exe': str((directory / 'exe').resolve()),
                'mount_namespace': str((directory / 'ns/mnt').readlink()),
                'libraries': paths,
                'deleted_dds_mapping': deleted,
            })
        except (OSError, ProcessLookupError):
            continue
    return {'captured_at_unix': time.time(), 'processes': processes}


if __name__ == '__main__':
    print(json.dumps(snapshot(), indent=2))
