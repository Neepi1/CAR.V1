#!/usr/bin/env python3
"""Build an approved API source snapshot cleanly; never install into the runtime.

Source selection is explicit: do not pass a dirty workspace unless all of its
changes have been reviewed for this deployment. No historical .o is reused.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import time


def manifest(root):
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(root.rglob('*')) if p.is_file()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--dependency-cache', type=Path)
    parser.add_argument('--jobs', type=int, default=2, choices=range(1, 5))
    args = parser.parse_args()
    source = args.source.resolve(strict=True)
    output = args.output.absolute()
    if source in output.parents or output == source:
        parser.error('output must be outside source')
    if not (source/'src/robot_api_server_node.cpp').is_file():
        parser.error('source must be the complete robot_api_server package')
    output.mkdir(parents=True, exist_ok=False)  # Old object reuse is prohibited.
    before = manifest(source)
    (output/'source_manifest.json').write_text(json.dumps(before, indent=2)+'\n')
    build = output/'build'
    stage = output/'stage'
    configure = ['cmake', '-S', str(source), '-B', str(build),
                 '-DCMAKE_BUILD_TYPE=RelWithDebInfo', '-DBUILD_TESTING=ON',
                 '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON', '-DCMAKE_INSTALL_PREFIX='+str(stage),
                 '-DAMENT_TEST_RESULTS_DIR='+str(output/'test_results')]
    if args.dependency_cache:
        for line in args.dependency_cache.read_text().splitlines():
            match = re.match(r'^([A-Za-z0-9_]+_DIR):(?:PATH|UNINITIALIZED)=(.+)$', line)
            if match:
                name, value = match.groups()
                dependency = Path(value)
                if dependency.is_dir() and (list(dependency.glob('*Config.cmake')) or
                                            list(dependency.glob('*-config.cmake'))):
                    configure.append('-D'+name+'='+value)
    started = time.time()
    subprocess.run(configure, check=True)
    subprocess.run(['cmake', '--build', str(build), '--target', 'robot_api_server_node',
                    '-j'+str(args.jobs)], check=True)
    commands = json.loads((build/'compile_commands.json').read_text())
    # Every package translation unit must come from this one immutable tree.
    for item in commands:
        if 'CMakeFiles/robot_api_server' in item['command']:
            if source not in Path(item['file']).resolve().parents:
                raise RuntimeError('translation unit outside approved source: '+item['file'])
    checked_dependencies = 0
    for depfile in build.glob('CMakeFiles/robot_api_server*.dir/**/*.o.d'):
        for token in shlex.split(depfile.read_text().replace('\\\n', ' ')):
            if '/include/robot_api_server/' in token and source not in Path(token).resolve().parents:
                raise RuntimeError('API header outside approved source: '+token)
        checked_dependencies += 1
    if checked_dependencies == 0:
        raise RuntimeError('no compiled dependency files available for header audit')
    if manifest(source) != before:
        raise RuntimeError('source changed during build; candidate rejected')
    subprocess.run(['cmake', '--install', str(build)], check=True)
    binary = stage/'lib/robot_api_server/robot_api_server_node'
    dependencies = subprocess.check_output(['ldd', str(binary)], text=True)
    (output/'ldd.txt').write_text(dependencies)
    if 'not found' in dependencies:
        raise RuntimeError('candidate has unresolved shared libraries')
    result = {'binary': str(binary), 'sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'source': str(source), 'translation_units': len(commands),
              'audited_object_dependencies': checked_dependencies,
              'build_seconds': round(time.time()-started, 3), 'production_installed': False,
              'restart_performed': False, 'tests_passed': False}
    (output/'candidate.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
