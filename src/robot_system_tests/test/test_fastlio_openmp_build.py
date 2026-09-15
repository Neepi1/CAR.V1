"""Exercise the actual CMake block shipped in the upstream OpenMP patch."""
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[3]
PATCH = ROOT / 'scripts/jetson/runtime_overlay/patches/fast_lio_openmp_threads.patch'


@pytest.mark.parametrize('threads,valid', [(None, True), ('1', True), ('4', True), ('0', False), ('-1', False), ('abc', False)])
def test_matching_compile_definitions(tmp_path, threads, valid):
    cmake = shutil.which('cmake')
    if not cmake:
        pytest.skip('CMake compiler environment required; exercised on Jetson')
    lines = PATCH.read_text().splitlines()[3:]
    block = '\n'.join(line[1:] for line in lines if line.startswith(('+', ' ')))
    (tmp_path/'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.16)\nproject(check_fastlio C CXX)\n' + block +
        '\nget_directory_property(defs COMPILE_DEFINITIONS)\nfile(WRITE "${CMAKE_BINARY_DIR}/defs.txt" "${defs}")\n')
    command = [cmake, '-S', str(tmp_path), '-B', str(tmp_path/'build'),
               '-DCMAKE_C_COMPILER=gcc', '-DCMAKE_CXX_COMPILER=g++']
    if threads is not None:
        command.append('-DFASTLIO_MATCH_THREADS='+threads)
    result = subprocess.run(command, text=True, capture_output=True, timeout=60)
    if not valid:
        assert result.returncode != 0
        assert 'must be a positive integer' in result.stderr
        return
    assert result.returncode == 0, result.stdout + result.stderr
    definitions = (tmp_path/'build/defs.txt').read_text().split(';')
    expected = threads or '4'
    assert 'MP_PROC_NUM='+expected in definitions
    assert ('MP_EN' in definitions) == (int(expected) > 1)
    assert sum(item.startswith('MP_PROC_NUM=') for item in definitions) == 1
