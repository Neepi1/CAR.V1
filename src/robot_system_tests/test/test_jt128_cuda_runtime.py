"""CUDA build/runtime contract; exercises only the config-generation fragment."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
OVERLAY = ROOT / "src/third_party/hesai_lidar_ros2_overlay"
RUN = ROOT / "scripts/jetson/runtime_overlay/scripts/run_driver.sh"


class CudaRuntimeContract(unittest.TestCase):
    def test_cuda_build_has_orin_release_and_humble_standard(self):
        text = (OVERLAY / "CMakeLists.txt").read_text(encoding="utf-8-sig")
        self.assertIn('option(FIND_CUDA "Build the Hesai CUDA point parser" ON)', text)
        self.assertIn('find_package(CUDA REQUIRED)', text)
        self.assertIn('set(CMAKE_CUDA_ARCHITECTURES "87" CACHE STRING', text)
        self.assertIn('set(CMAKE_CUDA_STANDARD 17)', text)
        for relative in ("CMakeLists.txt", "src/driver/HesaiLidar_SDK_2.0/CMakeLists.txt",
                         "src/driver/HesaiLidar_SDK_2.0/libhesai/CMakeLists.txt"):
            self.assertNotIn("set(CUDA_NVCC_FLAGS -arch=sm_61", (OVERLAY / relative).read_text(encoding="utf-8-sig"))

    def test_gpu_entry_preserves_current_cpu_entry(self):
        self.assertEqual((OVERLAY / "node/hesai_ros_driver_node.cc").read_text(encoding="utf-8-sig"),
                         (OVERLAY / "node/hesai_ros_driver_node.cu").read_text(encoding="utf-8-sig"))

    def test_runtime_gpu_toggle_keeps_other_fields(self):
        bash = "C:/Program Files/Git/bin/bash.exe" if os.name == "nt" else shutil.which("bash")
        self.assertTrue(bash)
        text = RUN.read_text(encoding="utf-8-sig")
        default = next(line for line in text.splitlines() if line.startswith("export NJRH_HESAI_USE_GPU="))
        fragment = text[text.index('RUNTIME_CONFIG_FILE="$(mktemp'):text.index('export CONFIG_FILE="${RUNTIME_CONFIG_FILE}"')]
        original = """lidar:
  - driver:
      use_gpu: false
      use_timestamp_type: 1
      echo_mode_filter: 2
      thread_num: 4
      frame_frequency: 20.0
      default_frame_frequency: 20.0
      transform_flag: false
      ros_frame_id: hesai_lidar
      ros_send_point_cloud_topic: /jt128/vendor/points_raw
      ros_send_imu_topic: /jt128/vendor/imu_raw
"""
        with tempfile.TemporaryDirectory(prefix="jt128_cuda_contract_") as directory:
            config = Path(directory) / "input.yaml"
            config.write_text(original, encoding="utf-8")
            for override, expected in ((None, "true"), ("false", "false")):
                env = dict(os.environ, CONFIG_FILE=config.as_posix(),
                           VENDOR_POINTS_TOPIC="/jt128/vendor/points_raw", VENDOR_IMU_TOPIC="/jt128/vendor/imu_raw")
                env.pop("NJRH_HESAI_USE_GPU", None)
                if override is not None:
                    env["NJRH_HESAI_USE_GPU"] = override
                script = 'set -eu\n' + default + '\n' + fragment + '\ncat "$RUNTIME_CONFIG_FILE"\nrm -f "$RUNTIME_CONFIG_FILE"\n'
                result = subprocess.run([bash, "-s"], input=script, text=True, capture_output=True,
                                        env=env, timeout=10, check=True)
                # Existing timestamp replacement appends its existing explanatory comment.
                def normalized(value):
                    return [line.split("#", 1)[0].rstrip() for line in value.splitlines()]
                self.assertEqual(normalized(result.stdout), normalized(original.replace("use_gpu: false", f"use_gpu: {expected}")))


if __name__ == "__main__":
    unittest.main()
