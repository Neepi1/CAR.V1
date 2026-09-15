#!/usr/bin/env bash
set -euo pipefail
# Build only. Does not install a package, source common_env, or start ROS.
[[ $# -eq 2 ]] || {
  echo "usage: $0 OFFICIAL_TARBALL REPORT_DIRECTORY" >&2; exit 2;
}
archive="$(realpath "$1")"
report="$(realpath "$2")"
[[ "$report" == /tmp/njrh_reports/* && -d "$report" ]]
expected=53476bcee331f28fe83e8387122fb7f84ab39209c459bd2cc227c71e4bd21c9b
printf '%s  %s\n' "$expected" "$archive" | sha256sum --check --status
[[ "$(dpkg --print-architecture)" == arm64 ]]
source_dir="$report/Fast-DDS-2.6.12"
if [[ ! -d "$source_dir" ]]; then
  tar -xzf "$archive" -C "$report"
fi
cmake -S "$source_dir" -B "$report/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/humble \
  -DCMAKE_PREFIX_PATH=/opt/ros/humble -DAPPEND_PROJECT_NAME_TO_INCLUDEDIR=ON \
  -DSECURITY=ON -DSQLITE3_SUPPORT=ON -DFASTDDS_STATISTICS=ON \
  -DCOMPILE_EXAMPLES=OFF -DCOMPILE_TOOLS=ON -DBUILD_TESTING=OFF \
  -DTHIRDPARTY_UPDATE=OFF > "$report/configure-package.log" 2>&1
nice -n 15 taskset -c 0,1,4 cmake --build "$report/build" \
  --parallel "${NJRH_DDS_BUILD_JOBS:-2}" > "$report/build-package.log" 2>&1

stage="$report/package-root"
[[ ! -e "$stage" ]] || { echo "package-root already exists; refusing overwrite" >&2; exit 1; }
mkdir -p "$stage/DEBIAN" "$stage/usr/share/doc/ros-humble-fastrtps"
DESTDIR="$stage" cmake --install "$report/build" > "$report/install-stage.log" 2>&1
[[ -f "$stage/opt/ros/humble/lib/libfastrtps.so.2.6.12" ]]
[[ ! -e "$stage/opt/ros/humble/lib/libfastrtps.so.2.6.10" ]]

# Keep the ROS package name so dpkg removes superseded files and owns the new
# headers, CMake exports, tools and library as one ordinary system upgrade.
depends="$(dpkg-query -W -f='${Depends}' ros-humble-fastrtps)"
size_kb="$(du -sk "$stage/opt" | awk '{print $1}')"
cat > "$stage/DEBIAN/control" <<EOF
Package: ros-humble-fastrtps
Version: 2.6.12-1njrh20260911
Architecture: arm64
Section: misc
Priority: optional
Maintainer: NJRH Runtime Build <runtime@localhost>
Installed-Size: $size_kb
Depends: $depends
Homepage: https://github.com/eProsima/Fast-DDS/releases/tag/v2.6.12
Description: Fast DDS 2.6.12 for the NJRH ROS 2 Humble runtime
 Local system-package build of unmodified official eProsima v2.6.12 sources.
 Includes upstream StatefulWriter lock-order-inversion fix PR 6463.
EOF
printf 'libfastrtps 2.6 ros-humble-fastrtps (>= 2.6.12)\n' > "$stage/DEBIAN/shlibs"
printf 'activate-noawait ldconfig\n' > "$stage/DEBIAN/triggers"
cp "$source_dir/LICENSE" "$stage/usr/share/doc/ros-humble-fastrtps/copyright"
printf 'Official source: https://github.com/eProsima/Fast-DDS/releases/tag/v2.6.12\nArchive SHA256: %s\nBuild UTC: %s\nSource modifications: none\n' \
  "$expected" "$(date -u +%FT%TZ)" > "$stage/usr/share/doc/ros-humble-fastrtps/njrh-build.txt"
(cd "$stage"; find opt usr -type f -print0 | sort -z | xargs -0 md5sum > DEBIAN/md5sums)
dpkg-deb --root-owner-group --build "$stage" \
  "$report/ros-humble-fastrtps_2.6.12-1njrh20260911_arm64.deb"
sha256sum "$report/ros-humble-fastrtps_2.6.12-1njrh20260911_arm64.deb" \
  "$stage/opt/ros/humble/lib/libfastrtps.so.2.6.12" > "$report/candidate-sha256.txt"
echo "Built candidate only; system package and running processes are unchanged."
