#!/usr/bin/env python3
"""No ROS: compare the reused matching loop with 1 and 4 OpenMP threads."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

PREFIX = r'''
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <ikd-Tree/ikd_Tree.h>
#include <omp.h>
#include <sched.h>
#include <array>
#include <iostream>
#include <stdexcept>
using namespace Eigen;
using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using V3D = Vector3d;
#define NUM_MATCH_POINTS 5
#define VF(n) Matrix<float, n, 1>
constexpr int feats_down_size = 2048;
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI), feats_down_world(new PointCloudXYZI), normvec(new PointCloudXYZI);
std::vector<PointVector> Nearest_Points(feats_down_size);
bool point_selected_surf[feats_down_size] = {};
float res_last[feats_down_size] = {};
KD_TREE<PointType> ikdtree;
struct State { Matrix3d rot=Matrix3d::Identity(), offset_R_L_I=Matrix3d::Identity(); V3D pos=V3D::Zero(), offset_T_L_I=V3D::Zero(); } s;
struct Data { bool converge = true; } ekfom_data;
'''
SUFFIX = r'''
using Result = std::array<double, 9>;
std::vector<Result> snapshot() {
  std::vector<Result> out;
  for (int i=0; i<feats_down_size; ++i) {
    const auto &p=feats_down_world->points[i], &n=normvec->points[i];
    out.push_back({p.x,p.y,p.z,n.x,n.y,n.z,n.intensity,res_last[i],double(point_selected_surf[i])});
    for (const auto &q : Nearest_Points[i]) out.push_back({q.x,q.y,q.z,0,0,0,0,0,0});
  }
  return out;
}
int main() {
  omp_set_dynamic(0);
  omp_set_num_threads(4);
  int team=0, bad_cpu=0;
  #pragma omp parallel reduction(+:bad_cpu)
  {
    #pragma omp single
    team=omp_get_num_threads();
    cpu_set_t mask; CPU_ZERO(&mask);
    if (sched_getaffinity(0, sizeof(mask), &mask) != 0 || CPU_COUNT(&mask)!=4 ||
        !CPU_ISSET(1,&mask) || !CPU_ISSET(2,&mask) || !CPU_ISSET(3,&mask) || !CPU_ISSET(4,&mask)) ++bad_cpu;
  }
  if (team!=4 || bad_cpu) throw std::runtime_error("OpenMP team/affinity mismatch");
  PointVector map;
  for(int x=0;x<100;++x) for(int y=0;y<100;++y) {
    PointType p{}; p.x=0.05f*x; p.y=0.05f*y; p.z=1.f; map.push_back(p);
    p.z=0.05f*y; p.y=5.f; map.push_back(p);
  }
  ikdtree.Build(map);
  feats_down_body->resize(feats_down_size); feats_down_world->resize(feats_down_size); normvec->resize(feats_down_size);
  for(int i=0;i<feats_down_size;++i) {
    auto &p=feats_down_body->points[i];
    p=PointType{}; p.x=0.03f+0.0461f*(i%99); p.y=0.03f+0.0473f*((i*37)%99); p.z=1.015f;
  }
  match(1); auto expected=snapshot();
  int selected=0; for(bool b:point_selected_surf) selected+=b;
  if(selected<1000) throw std::runtime_error("Insufficient matched points");
  double max_error=0.;
  for(int repeat=0;repeat<12;++repeat) {
    // Include both fresh nearest-neighbor queries and cached correspondences.
    ekfom_data.converge = repeat % 2 == 0;
    match(4); auto actual=snapshot();
    if(actual.size()!=expected.size()) throw std::runtime_error("Match count mismatch");
    for(size_t i=0;i<actual.size();++i) for(size_t j=0;j<actual[i].size();++j) {
      double error=std::abs(actual[i][j]-expected[i][j]);
      if(!std::isfinite(error) || error>1e-6) throw std::runtime_error("Parallel result mismatch");
      max_error=std::max(max_error,error);
    }
  }
  std::cout << "PASS team=" << team << " affinity=1-4 points=" << feats_down_size
            << " selected=" << selected << " repeats=12 max_error=" << max_error << std::endl;
}
'''

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    laser = args.source / 'src/laserMapping.cpp'
    common = args.source / 'include/common_lib.h'
    code = laser.read_text()
    begin = code.index('    #ifdef MP_EN', code.index('void h_share_model('))
    end = code.index('    effct_feat_num = 0;', begin)
    loop = code[begin:end]
    assert loop.count('omp_set_num_threads(MP_PROC_NUM)') == 1
    assert loop.count('#pragma omp parallel for') == 1
    loop = loop.replace('omp_set_num_threads(MP_PROC_NUM)', 'omp_set_num_threads(threads)')
    common_code = common.read_text()
    begin = common_code.index('template<typename T>\nbool esti_plane(')
    end = common_code.index('\ndouble get_time_sec(', begin)
    harness = args.output_dir / 'matching_equivalence.cpp'
    harness.write_text(PREFIX + common_code[begin:end] + '\nvoid match(int threads) {\n' + loop + '\n}\n' + SUFFIX)
    executable = args.output_dir / 'matching_equivalence'
    # Jetson Humble image has PCL development files but no pkg-config .pc file.
    command = ['g++', '-O2', '-std=c++17', '-fopenmp', '-pthread', '-DMP_EN', '-I'+str(args.source/'include'),
               '-I/usr/include/pcl-1.12', '-I/usr/include/eigen3', str(harness),
               str(args.source/'include/ikd-Tree/ikd_Tree.cpp'), '-o', str(executable), '-lpcl_common']
    subprocess.run(command, check=True, timeout=180)
    result = subprocess.run(['taskset', '-c', '1-4', str(executable)], capture_output=True, text=True, timeout=30)
    (args.output_dir/'matching_test.json').write_text(json.dumps(dict(
        source_sha256=hashlib.sha256(laser.read_bytes()).hexdigest(), compile_command=command,
        returncode=result.returncode, stdout=result.stdout, stderr=result.stderr), indent=2))
    print(result.stdout, end='')
    print(result.stderr, end='')
    result.check_returncode()

if __name__ == '__main__':
    main()
