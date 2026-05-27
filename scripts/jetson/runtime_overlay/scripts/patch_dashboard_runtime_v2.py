#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import patch_dashboard_runtime as base


LOCAL_COSTMAP_DEBUG_METHODS = """    def _local_costmap_debug_patterns(self) -> List[str]:
        return [
            'run_local_costmap_debug.sh',
            'ros2 launch .*local_costmap_debug.launch.py',
            'local_costmap_debug.launch.py',
            'lifecycle_manager_local_costmap_debug',
        ]

    def _local_costmap_debug_running(self) -> bool:
        return (
            self._process_exists('run_local_costmap_debug.sh') or
            self._process_exists('local_costmap_debug.launch.py') or
            self._process_exists('lifecycle_manager_local_costmap_debug') or
            self._process_exists('controller_server')
        )

    def _local_costmap_debug_active(self) -> bool:
        lifecycle = self._ros_cli_text(
            ['bash', '-lc', 'source /opt/ros/humble/setup.bash && ros2 lifecycle get /local_costmap/local_costmap'],
            timeout=8.0,
        ).lower()
        return 'active [3]' in lifecycle or self.ros_state.latest_local_costmap_payload() is not None

    def start_local_costmap_debug(self) -> dict:
        with self._operation_lock:
            try:
                self._trace('start_local_costmap_debug begin')
                actions: List[str] = []
                stopped_navigation_mode = False
                if self._nav2_localization_running():
                    self._stop('nav2_localization')
                    self._stop('relocalization')
                    self._stop('map_align')
                    self._kill_patterns(self._nav2_localization_patterns())
                    actions.append('existing 2D localization stack stopped for local costmap debug')
                    stopped_navigation_mode = True
                if self._nav2_navigation_running():
                    self._stop('nav2_navigation')
                    self._stop('nav2')
                    self._kill_patterns(self._nav2_navigation_patterns())
                    actions.append('existing Nav2 control stopped for local costmap debug')
                    stopped_navigation_mode = True
                if stopped_navigation_mode:
                    self._clear_navigation_display_anchor()
                    with self._lock:
                        self._nav2_map_name = None
                        self._nav2_profile = None
                        self._nav2_activation_token += 1
                self._stop('local_costmap_debug')
                self._kill_patterns(self._local_costmap_debug_patterns())
                self.ros_state.clear_local_costmap_cache()
                actions.extend(self._ensure_driver_ready('mapping'))
                actions.extend(self._ensure_chassis_ready(
                    publish_odom_tf=False,
                    force_restart=False,
                    wait_for_chassis_odometry=False,
                ))
                self._start(
                    'local_costmap_debug',
                    ['bash', '-lc', 'NAV2_PARAMS_FILE=config/nav2.yaml bash scripts/run_local_costmap_debug.sh'],
                )
                actions.append('local costmap debug stack start requested')
                with self._temporary_view_features(['local_costmap_live'], source='internal:local_costmap_debug'):
                    self._wait_until(
                        lambda: self._local_costmap_debug_running(),
                        timeout=25.0,
                        description='local costmap debug process startup',
                    )
                    self._wait_until(
                        lambda: self._local_costmap_debug_active(),
                        timeout=35.0,
                        description='local costmap lifecycle activation',
                    )
                    self._wait_until(
                        lambda: self.ros_state.latest_local_costmap_payload() is not None,
                        timeout=20.0,
                        description='/local_costmap/costmap data',
                    )
                actions.append('local costmap debug ready')
                self._trace('start_local_costmap_debug success')
                return {'ok': True, 'message': ', '.join(actions)}
            except Exception as exc:
                self._trace(f'start_local_costmap_debug exception: {exc}')
                return {'ok': False, 'error': str(exc)}

    def stop_local_costmap_debug(self) -> dict:
        with self._operation_lock:
            self._trace('stop_local_costmap_debug begin')
            self._stop('local_costmap_debug')
            self._kill_patterns(self._local_costmap_debug_patterns() + ['controller_server'])
            self.ros_state.clear_local_costmap_cache()
            self._trace('stop_local_costmap_debug finished')
            return {'ok': True, 'message': 'Local costmap debug stack stopped'}

"""


FLOOR_TESTING_METHODS = """    @staticmethod
    def _sanitize_floor_token(value: str, label: str) -> str:
        token = str(value or '').strip()
        if not token:
            raise RuntimeError(f'{label} is required')
        allowed = set('abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-')
        if any(ch not in allowed for ch in token):
            raise RuntimeError(f'{label} may only contain letters, digits, _ and -')
        return token

    def _floor_release_assets_dir(self) -> Path:
        project_root = self.env.get('NJRH_PROJECT_ROOT') or os.environ.get(
            'NJRH_PROJECT_ROOT',
            '/workspaces/njrh-v3/workspace1',
        )
        return Path(self.env.get('NJRH_RELEASE_ASSETS_DIR') or os.environ.get(
            'NJRH_RELEASE_ASSETS_DIR',
            str(Path(project_root) / 'maps_release'),
        ))

    def _floor_required_assets(self) -> List[str]:
        return [
            'nav/nav_map.yaml',
            'nav/nav_map.pgm',
            'localizer/localizer_map.png',
            'localizer/localizer_params.yaml',
            'filters/keepout_mask.yaml',
            'filters/keepout_mask.pgm',
            'filters/speed_mask.yaml',
            'filters/speed_mask.pgm',
            'filters/binary_mask.yaml',
            'filters/binary_mask.pgm',
            'reports/asset_report.json',
            'poses.yaml',
        ]

    def _floor_bundle_snapshot(self, building_dir: Path, floor_dir: Path) -> dict:
        missing = [
            str(floor_dir / rel)
            for rel in self._floor_required_assets()
            if not (floor_dir / rel).is_file()
        ]
        report_path = floor_dir / 'reports' / 'asset_report.json'
        report = {}
        if report_path.is_file():
            try:
                report = json.loads(report_path.read_text(encoding='utf-8'))
            except Exception as exc:
                report = {'error': f'failed to parse asset_report.json: {exc}'}
        return {
            'building_id': building_dir.name,
            'floor_id': floor_dir.name,
            'floor_root': str(floor_dir),
            'valid': not missing,
            'missing': missing,
            'nav_map': str(floor_dir / 'nav' / 'nav_map.yaml'),
            'localizer_map': str(floor_dir / 'localizer' / 'localizer_params.yaml'),
            'asset_report': report,
        }

    def list_floor_assets(self) -> dict:
        release_root = self._floor_release_assets_dir()
        floors = []
        if release_root.is_dir():
            for building_dir in sorted(path for path in release_root.iterdir() if path.is_dir()):
                for floor_dir in sorted(path for path in building_dir.iterdir() if path.is_dir()):
                    floors.append(self._floor_bundle_snapshot(building_dir, floor_dir))
        with self._lock:
            selected = {
                'building_id': self.env.get('NJRH_BUILDING_ID', ''),
                'floor_id': self.env.get('NJRH_FLOOR_ID', ''),
                'floor_root': self.env.get('NJRH_CURRENT_FLOOR_ROOT', ''),
                'nav_map': self.env.get('NAV2_MAP_YAML', ''),
                'localizer_map': self.env.get('NAV2_LOCALIZER_MAP_YAML', ''),
            }
        return {
            'ok': True,
            'root': str(release_root),
            'selected': selected,
            'floors': floors,
        }

    def _run_floor_script(self, script_name: str, args: List[str], timeout: float) -> str:
        script_path = self.root_dir / 'scripts' / script_name
        if not script_path.is_file():
            raise RuntimeError(f'floor test helper missing: {script_path}')
        completed = subprocess.run(
            ['bash', str(script_path), *[str(arg) for arg in args]],
            cwd=str(self.root_dir),
            env=self.env,
            capture_output=True,
            text=True,
            timeout=float(timeout),
            check=False,
        )
        output = '\\n'.join(
            part.strip()
            for part in (completed.stdout or '', completed.stderr or '')
            if part and part.strip()
        ).strip()
        if completed.returncode != 0:
            raise RuntimeError(output or f'{script_name} failed with rc={completed.returncode}')
        return output

    def _parse_floor_export_env(self, output: str) -> dict:
        parsed = {}
        for raw_line in str(output or '').splitlines():
            line = raw_line.strip()
            if not line.startswith('export ') or '=' not in line:
                continue
            key, value = line[len('export '):].split('=', 1)
            key = key.strip()
            value = value.strip()
            if len(value) >= 2 and value[0] == value[-1] == "'":
                value = value[1:-1]
            if key:
                parsed[key] = value
        return parsed

    def promote_map_to_floor(self, map_name: str, building_id: str, floor_id: str) -> dict:
        try:
            safe_map = self._sanitize_asset_name(map_name, 'Invalid map name')
            safe_building = self._sanitize_floor_token(building_id, 'building_id')
            safe_floor = self._sanitize_floor_token(floor_id, 'floor_id')
            output = self._run_floor_script(
                'promote_map_to_floor.sh',
                [safe_map, safe_building, safe_floor],
                timeout=180.0,
            )
            return {
                'ok': True,
                'message': f'测试：已将地图 {safe_map} 归档到 {safe_building}/{safe_floor}',
                'details': output,
                'floors': self.list_floor_assets(),
            }
        except Exception as exc:
            return {'ok': False, 'error': str(exc)}

    def select_floor_assets(self, building_id: str, floor_id: str) -> dict:
        try:
            safe_building = self._sanitize_floor_token(building_id, 'building_id')
            safe_floor = self._sanitize_floor_token(floor_id, 'floor_id')
            output = self._run_floor_script(
                'select_floor_assets.sh',
                [safe_building, safe_floor],
                timeout=30.0,
            )
            env_updates = self._parse_floor_export_env(output)
            with self._lock:
                self.env.update(env_updates)
            os.environ.update(env_updates)
            return {
                'ok': True,
                'message': f'测试：已选择楼层资产 {safe_building}/{safe_floor}，后续 Web 启动流程会继承该选择',
                'details': output,
                'selected': {
                    'building_id': env_updates.get('NJRH_BUILDING_ID', safe_building),
                    'floor_id': env_updates.get('NJRH_FLOOR_ID', safe_floor),
                    'floor_root': env_updates.get('NJRH_CURRENT_FLOOR_ROOT', ''),
                    'nav_map': env_updates.get('NAV2_MAP_YAML', ''),
                    'localizer_map': env_updates.get('NAV2_LOCALIZER_MAP_YAML', ''),
                },
            }
        except Exception as exc:
            return {'ok': False, 'error': str(exc)}

    def _floor_manager_patterns(self) -> List[str]:
        return [
            'run_floor_manager.sh',
            'floor_manager_node',
            'robot_floor_manager/floor_manager_node',
        ]

    def _floor_switch_service_available(self) -> bool:
        output = self._ros_cli_text(
            ['bash', '-lc', 'source /opt/ros/humble/setup.bash && ros2 service list'],
            timeout=6.0,
        )
        return '/floor_manager/switch_floor' in output

    def _ensure_floor_manager_ready(self) -> List[str]:
        actions = []
        if not self._floor_switch_service_available():
            self._start('floor_manager', ['bash', '-lc', 'bash scripts/run_floor_manager.sh'])
            actions.append('floor manager start requested')
            self._wait_until(
                self._floor_switch_service_available,
                timeout=25.0,
                description='/floor_manager/switch_floor service',
            )
        else:
            actions.append('floor manager already running')
        return actions

    def switch_floor(self, building_id: str, floor_id: str) -> dict:
        with self._operation_lock:
            try:
                safe_building = self._sanitize_floor_token(building_id, 'building_id')
                safe_floor = self._sanitize_floor_token(floor_id, 'floor_id')
                actions = []
                select_result = self.select_floor_assets(safe_building, safe_floor)
                if not select_result.get('ok'):
                    return select_result
                actions.append(select_result.get('message', 'floor assets selected'))
                actions.extend(self._ensure_floor_manager_ready())
                payload = "{building_id: '" + safe_building + "', floor_id: '" + safe_floor + "', resume_navigation: false}"
                ros_command = (
                    'PROJECT_ROOT="${NJRH_PROJECT_ROOT:-/workspaces/njrh-v3/workspace1}"; '
                    'cd "${PROJECT_ROOT}" && '
                    'source /opt/ros/humble/setup.bash && '
                    '[ ! -f install/local_setup.bash ] || source install/local_setup.bash; '
                    f'ros2 service call /floor_manager/switch_floor robot_interfaces/srv/SwitchFloor {shlex.quote(payload)}'
                )
                completed = subprocess.run(
                    ['bash', '-lc', ros_command],
                    cwd=str(self.root_dir),
                    env=self.env,
                    capture_output=True,
                    text=True,
                    timeout=120.0,
                    check=False,
                )
                output = '\\n'.join(
                    part.strip()
                    for part in (completed.stdout or '', completed.stderr or '')
                    if part and part.strip()
                ).strip()
                if completed.returncode != 0:
                    raise RuntimeError(output or 'floor switch service call failed')
                lowered = output.lower()
                if 'success=false' in lowered or 'success: false' in lowered:
                    raise RuntimeError(output or 'floor switch service returned failure')
                actions.append('floor switch service call completed')
                return {
                    'ok': True,
                    'message': '测试：' + ', '.join(actions),
                    'details': output,
                }
            except Exception as exc:
                return {'ok': False, 'error': str(exc)}

"""


def patch_dashboard_server_runtime_fixed(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    text = base.replace_once(
        text,
        "            'ros2 launch .*navigation_launch.py',\n",
        "            'ros2 launch .*navigation_launch.py',\n            'ros2 launch .*standard_navigation.launch.py',\n",
        "standard navigation kill pattern",
    )
    text = base.replace_once(
        text,
        "            'local_costmap_grid': {\n                'msg_type': OccupancyGrid,\n                'topic': '/local_costmap/costmap',\n                'callback': self._on_local_costmap_grid,\n                'qos': 10,\n                'callback_group': self._default_callback_group,\n                'clear': self.clear_local_costmap_cache,\n            },\n",
        "            'local_costmap_grid': {\n                'msg_type': OccupancyGrid,\n                'topic': '/local_costmap/costmap',\n                'callback': self._on_local_costmap_grid,\n                'qos': self._map_qos,\n                'callback_group': self._default_callback_group,\n                'clear': self.clear_local_costmap_cache,\n            },\n",
        "local_costmap subscription qos",
    )
    text = base.replace_once(
        text,
        "    def _nav2_params_path_for_profile(self, profile: str) -> Path:\n        if profile == 'rapid_avoidance':\n            return self.root_dir / 'nav2_test' / 'params' / 'nav2_jt128_rapid_avoidance.yaml'\n        return self.root_dir / 'nav2_test' / 'params' / 'nav2_jt128_2d_global_nvblox_local.yaml'\n",
        "    def _nav2_params_path_for_profile(self, profile: str) -> Path:\n        return self.root_dir / 'config' / 'nav2.yaml'\n",
        "nav2 params path override",
    )
    text = base.replace_once(
        text,
        "    def _serve_file(self, file_name: str, content_type: str) -> None:\n        file_path = STATIC_DIR / file_name\n        if not file_path.exists():\n            self.send_error(HTTPStatus.NOT_FOUND)\n            return\n        data = file_path.read_bytes()\n        try:\n            self.send_response(HTTPStatus.OK)\n            self.send_header('Content-Type', content_type)\n            self.send_header('Content-Length', str(len(data)))\n            self.end_headers()\n            self.wfile.write(data)\n        except Exception as exc:\n            if not self._is_client_disconnect(exc):\n                raise\n",
        "    def _serve_file(self, file_name: str, content_type: str) -> None:\n        file_path = STATIC_DIR / file_name\n        if not file_path.exists():\n            self.send_error(HTTPStatus.NOT_FOUND)\n            return\n        data = file_path.read_bytes()\n        try:\n            self.send_response(HTTPStatus.OK)\n            self.send_header('Content-Type', content_type)\n            self.send_header('Content-Length', str(len(data)))\n            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')\n            self.send_header('Pragma', 'no-cache')\n            self.send_header('Expires', '0')\n            self.end_headers()\n            self.wfile.write(data)\n        except Exception as exc:\n            if not self._is_client_disconnect(exc):\n                raise\n",
        "_serve_file no-cache",
    )
    text = base.replace_once(
        text,
        "    def _serve_static_path(self, relative_path: str) -> None:\n        safe_parts = [part for part in Path(relative_path).parts if part not in ('', '.', '..')]\n        file_path = STATIC_DIR.joinpath(*safe_parts)\n        if not file_path.exists() or not file_path.is_file():\n            self.send_error(HTTPStatus.NOT_FOUND)\n            return\n\n        suffix = file_path.suffix.lower()\n        content_types = {\n            '.html': 'text/html; charset=utf-8',\n            '.js': 'text/javascript; charset=utf-8',\n            '.mjs': 'text/javascript; charset=utf-8',\n            '.css': 'text/css; charset=utf-8',\n            '.json': 'application/json; charset=utf-8',\n            '.map': 'application/json; charset=utf-8',\n        }\n        content_type = content_types.get(suffix, 'application/octet-stream')\n        data = file_path.read_bytes()\n        try:\n            self.send_response(HTTPStatus.OK)\n            self.send_header('Content-Type', content_type)\n            self.send_header('Content-Length', str(len(data)))\n            self.end_headers()\n            self.wfile.write(data)\n        except Exception as exc:\n            if not self._is_client_disconnect(exc):\n                raise\n",
        "    def _serve_static_path(self, relative_path: str) -> None:\n        safe_parts = [part for part in Path(relative_path).parts if part not in ('', '.', '..')]\n        file_path = STATIC_DIR.joinpath(*safe_parts)\n        if not file_path.exists() or not file_path.is_file():\n            self.send_error(HTTPStatus.NOT_FOUND)\n            return\n\n        suffix = file_path.suffix.lower()\n        content_types = {\n            '.html': 'text/html; charset=utf-8',\n            '.js': 'text/javascript; charset=utf-8',\n            '.mjs': 'text/javascript; charset=utf-8',\n            '.css': 'text/css; charset=utf-8',\n            '.json': 'application/json; charset=utf-8',\n            '.map': 'application/json; charset=utf-8',\n        }\n        content_type = content_types.get(suffix, 'application/octet-stream')\n        data = file_path.read_bytes()\n        try:\n            self.send_response(HTTPStatus.OK)\n            self.send_header('Content-Type', content_type)\n            self.send_header('Content-Length', str(len(data)))\n            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')\n            self.send_header('Pragma', 'no-cache')\n            self.send_header('Expires', '0')\n            self.end_headers()\n            self.wfile.write(data)\n        except Exception as exc:\n            if not self._is_client_disconnect(exc):\n                raise\n",
        "_serve_static_path no-cache",
    )
    text = base.replace_once(
        text,
        "    def activate_view_lease(\n        self,\n        feature_names: List[str],\n        *,\n        ttl: float = VIEW_LEASE_TTL_SEC,\n        source: str = 'http',\n    ) -> dict:\n        features = self._normalize_view_features(feature_names)\n        subscriptions = self._expand_view_features(features)\n        now = time.time()\n        with self._lock:\n            self._view_lease_seq += 1\n            lease_id = f'lease-{self._view_lease_seq}'\n            self._view_leases[lease_id] = {\n                'features': list(features),\n                'subscriptions': list(subscriptions),\n                'source': str(source or 'http'),\n                'created_at': now,\n                'updated_at': now,\n                'expires_at': now + max(1.0, float(ttl)),\n            }\n            self._subscriptions_dirty = True\n        return {\n            'ok': True,\n            'lease_id': lease_id,\n            'features': list(features),\n            'subscriptions': list(subscriptions),\n            'expires_at': now + max(1.0, float(ttl)),\n        }\n",
        "    def activate_view_lease(\n        self,\n        feature_names: List[str],\n        *,\n        ttl: float = VIEW_LEASE_TTL_SEC,\n        source: str = 'http',\n    ) -> dict:\n        features = self._normalize_view_features(feature_names)\n        subscriptions = self._expand_view_features(features)\n        now = time.time()\n        with self._lock:\n            self._view_lease_seq += 1\n            lease_id = f'lease-{self._view_lease_seq}'\n            self._view_leases[lease_id] = {\n                'features': list(features),\n                'subscriptions': list(subscriptions),\n                'source': str(source or 'http'),\n                'created_at': now,\n                'updated_at': now,\n                'expires_at': now + max(1.0, float(ttl)),\n            }\n            self._subscriptions_dirty = True\n        self._refresh_dynamic_subscriptions()\n        return {\n            'ok': True,\n            'lease_id': lease_id,\n            'features': list(features),\n            'subscriptions': list(subscriptions),\n            'expires_at': now + max(1.0, float(ttl)),\n        }\n",
        "activate_view_lease immediate refresh",
    )
    text = base.replace_once(
        text,
        "    def heartbeat_view_lease(\n        self,\n        lease_id: str,\n        *,\n        ttl: float = VIEW_LEASE_TTL_SEC,\n        feature_names: Optional[List[str]] = None,\n        source: str = 'http',\n    ) -> dict:\n        safe_lease_id = str(lease_id or '').strip()\n        if not safe_lease_id:\n            raise RuntimeError('Missing view lease id')\n        now = time.time()\n        with self._lock:\n            lease = self._view_leases.get(safe_lease_id)\n            if lease is None:\n                raise RuntimeError(f'Unknown view lease: {safe_lease_id}')\n            if feature_names is not None:\n                features = self._normalize_view_features(feature_names)\n                lease['features'] = list(features)\n                lease['subscriptions'] = self._expand_view_features(features)\n            lease['updated_at'] = now\n            lease['source'] = str(source or lease.get('source') or 'http')\n            lease['expires_at'] = now + max(1.0, float(ttl))\n            self._subscriptions_dirty = True\n            features = list(lease.get('features', []))\n            subscriptions = list(lease.get('subscriptions', []))\n            expires_at = float(lease.get('expires_at', now))\n        return {\n            'ok': True,\n            'lease_id': safe_lease_id,\n            'features': features,\n            'subscriptions': subscriptions,\n            'expires_at': expires_at,\n        }\n",
        "    def heartbeat_view_lease(\n        self,\n        lease_id: str,\n        *,\n        ttl: float = VIEW_LEASE_TTL_SEC,\n        feature_names: Optional[List[str]] = None,\n        source: str = 'http',\n    ) -> dict:\n        safe_lease_id = str(lease_id or '').strip()\n        if not safe_lease_id:\n            raise RuntimeError('Missing view lease id')\n        now = time.time()\n        with self._lock:\n            lease = self._view_leases.get(safe_lease_id)\n            if lease is None:\n                raise RuntimeError(f'Unknown view lease: {safe_lease_id}')\n            if feature_names is not None:\n                features = self._normalize_view_features(feature_names)\n                lease['features'] = list(features)\n                lease['subscriptions'] = self._expand_view_features(features)\n            lease['updated_at'] = now\n            lease['source'] = str(source or lease.get('source') or 'http')\n            lease['expires_at'] = now + max(1.0, float(ttl))\n            self._subscriptions_dirty = True\n            features = list(lease.get('features', []))\n            subscriptions = list(lease.get('subscriptions', []))\n            expires_at = float(lease.get('expires_at', now))\n        self._refresh_dynamic_subscriptions()\n        return {\n            'ok': True,\n            'lease_id': safe_lease_id,\n            'features': features,\n            'subscriptions': subscriptions,\n            'expires_at': expires_at,\n        }\n",
        "heartbeat_view_lease immediate refresh",
    )
    text = base.replace_once(
        text,
        "    def release_view_lease(self, lease_id: str) -> dict:\n        safe_lease_id = str(lease_id or '').strip()\n        if not safe_lease_id:\n            return {'ok': True, 'released': False}\n        with self._lock:\n            removed = self._view_leases.pop(safe_lease_id, None)\n            if removed is not None:\n                self._subscriptions_dirty = True\n        return {'ok': True, 'released': removed is not None, 'lease_id': safe_lease_id}\n",
        "    def release_view_lease(self, lease_id: str) -> dict:\n        safe_lease_id = str(lease_id or '').strip()\n        if not safe_lease_id:\n            return {'ok': True, 'released': False}\n        with self._lock:\n            removed = self._view_leases.pop(safe_lease_id, None)\n            if removed is not None:\n                self._subscriptions_dirty = True\n        self._refresh_dynamic_subscriptions()\n        return {'ok': True, 'released': removed is not None, 'lease_id': safe_lease_id}\n",
        "release_view_lease immediate refresh",
    )
    text = base.replace_once(
        text,
        "    def _nav2_navigation_nodes_running(self) -> bool:\n        return (\n            self._process_exists('planner_server') or\n            self._process_exists('controller_server') or\n            self._process_exists('bt_navigator') or\n            self._process_exists('behavior_server') or\n            self._process_exists('lifecycle_manager_navigation')\n        )\n\n    def _resolve_navigation_map_yaml(self, map_name: str = '') -> Path:\n",
        "    def _nav2_navigation_nodes_running(self) -> bool:\n        return (\n            self._process_exists('planner_server') or\n            self._process_exists('controller_server') or\n            self._process_exists('bt_navigator') or\n            self._process_exists('behavior_server') or\n            self._process_exists('lifecycle_manager_navigation')\n        )\n\n    @staticmethod\n    def _safe_cli_text(value) -> str:\n        if value is None:\n            return ''\n        if isinstance(value, bytes):\n            return value.decode('utf-8', errors='replace')\n        return str(value)\n\n    def _ros_cli_text(self, command: List[str], timeout: float = 12.0) -> str:\n        try:\n            completed = subprocess.run(\n                command,\n                capture_output=True,\n                text=True,\n                timeout=max(2.0, float(timeout)),\n                env=os.environ.copy(),\n                check=False,\n            )\n        except subprocess.TimeoutExpired as exc:\n            return '\\n'.join(part for part in (self._safe_cli_text(exc.stdout), self._safe_cli_text(exc.stderr)) if part).strip()\n        return '\\n'.join(part for part in (self._safe_cli_text(completed.stdout), self._safe_cli_text(completed.stderr)) if part).strip()\n\n    def _nav2_navigation_nodes_active(self) -> bool:\n        node_names = [\n            '/controller_server',\n            '/planner_server',\n            '/bt_navigator',\n            '/local_costmap/local_costmap',\n            '/global_costmap/global_costmap',\n            '/velocity_smoother',\n            '/collision_monitor',\n        ]\n        for node_name in node_names:\n            output = self._ros_cli_text(\n                ['bash', '-lc', f'source /opt/ros/humble/setup.bash && ros2 lifecycle get {shlex.quote(node_name)}'],\n                timeout=8.0,\n            ).lower()\n            if 'active [3]' not in output:\n                return False\n        return True\n\n    def _startup_nav2_lifecycle_manager(self) -> str:\n        output = self._ros_cli_text(\n            [\n                'bash',\n                '-lc',\n                \"source /opt/ros/humble/setup.bash && ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 0}'\",\n            ],\n            timeout=20.0,\n        )\n        lowered = output.lower()\n        if 'success=true' not in lowered and 'success: true' not in lowered:\n            if self._nav2_navigation_nodes_active():\n                return 'Nav2 lifecycle already active'\n            raise RuntimeError(\n                'Failed to trigger Nav2 lifecycle startup via /lifecycle_manager_navigation/manage_nodes'\n                f' ({output or \"no response\"})'\n            )\n        return 'Nav2 lifecycle startup requested'\n\n    def _resolve_navigation_map_yaml(self, map_name: str = '') -> Path:\n",
        "nav2 navigation lifecycle helpers",
    )
    text = base.replace_once(
        text,
        "    def _start_navigation_nav_stack(self, nav2_params: Optional[Path] = None, nav_profile: str = 'standard') -> List[str]:\n        actions: List[str] = []\n        nav2_params = nav2_params or self._nav2_params_path_for_profile(nav_profile)\n        desired_command = self._nav2_command_for_profile(nav_profile, nav2_params)\n        proc = self.processes.get('nav2_navigation')\n        needs_restart = proc is None or proc.command != desired_command or not proc.is_running()\n        if needs_restart:\n            self._stop('nav2_navigation')\n            self._stop('nav2')\n            self._kill_patterns(self._nav2_navigation_patterns())\n            self._start('nav2_navigation', desired_command)\n            actions.append(f'{self._nav2_profile_label(nav_profile)} stack start requested')\n        else:\n            actions.append(f'{self._nav2_profile_label(nav_profile)} stack already running')\n        self._wait_until(\n            lambda: self._nav2_navigation_nodes_running(),\n            timeout=35.0,\n            description='Nav2 navigation nodes startup',\n        )\n        actions.append(f'{self._nav2_profile_label(nav_profile)} stack ready')\n        return actions\n",
        "    def _start_navigation_nav_stack(self, nav2_params: Optional[Path] = None, nav_profile: str = 'standard') -> List[str]:\n        actions: List[str] = []\n        nav2_params = nav2_params or self._nav2_params_path_for_profile(nav_profile)\n        desired_command = self._nav2_command_for_profile(nav_profile, nav2_params)\n        proc = self.processes.get('nav2_navigation')\n        needs_restart = proc is None or proc.command != desired_command or not proc.is_running()\n        if needs_restart:\n            self._stop('nav2_navigation')\n            self._stop('nav2')\n            self._kill_patterns(self._nav2_navigation_patterns())\n            self._start('nav2_navigation', desired_command)\n            actions.append(f'{self._nav2_profile_label(nav_profile)} stack start requested')\n        else:\n            actions.append(f'{self._nav2_profile_label(nav_profile)} stack already running')\n        self._wait_until(\n            lambda: self._nav2_navigation_nodes_running(),\n            timeout=35.0,\n            description='Nav2 navigation nodes startup',\n        )\n        if not self._nav2_navigation_nodes_active():\n            try:\n                actions.append(self._startup_nav2_lifecycle_manager())\n            except Exception as exc:\n                self._wait_until(\n                    lambda: self._nav2_navigation_nodes_active(),\n                    timeout=35.0,\n                    description='Nav2 navigation lifecycle activation after startup retry',\n                )\n                actions.append(f'Nav2 lifecycle already active after startup retry ({exc})')\n            self._wait_until(\n                lambda: self._nav2_navigation_nodes_active(),\n                timeout=25.0,\n                description='Nav2 navigation lifecycle activation',\n            )\n        actions.append(f'{self._nav2_profile_label(nav_profile)} stack ready')\n        return actions\n",
        "nav2 navigation lifecycle activation",
    )
    path.write_text(text, encoding="utf-8")


def patch_dashboard_server_local_costmap_debug(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    if "'run_local_costmap_debug.sh'," not in text:
        text = base.replace_once(
            text,
            "            'run_nav2_rapid_avoidance.sh',\n",
            "            'run_nav2_rapid_avoidance.sh',\n            'run_local_costmap_debug.sh',\n            'ros2 launch .*local_costmap_debug.launch.py',\n            'local_costmap_debug.launch.py',\n            'lifecycle_manager_local_costmap_debug',\n",
            "local costmap debug nav kill patterns",
        )
    if "def start_local_costmap_debug(self) -> dict:" not in text:
        text = base.replace_once(
            text,
            "    def start_nvblox(self) -> dict:\n",
            LOCAL_COSTMAP_DEBUG_METHODS + "    def start_nvblox(self) -> dict:\n",
            "local costmap debug manager methods",
        )
    if "self._stop('local_costmap_debug')" not in text.split("    def stop_navigation(self) -> dict:", 1)[1].split("    def stop_core(self) -> dict:", 1)[0]:
        text = base.replace_once(
            text,
            "            self._stop('nav2_localization')\n            self._stop('nav2_navigation')\n            self._stop('nav2')\n            self._stop('relocalization')\n",
            "            self._stop('nav2_localization')\n            self._stop('nav2_navigation')\n            self._stop('nav2')\n            self._stop('local_costmap_debug')\n            self._stop('relocalization')\n",
            "stop_navigation local costmap debug stop",
        )
    if "/api/local_costmap_debug/start" not in text:
        text = base.replace_once(
            text,
            "        if parsed.path == '/api/core/stop':\n            self.manager._trace('HTTP /api/core/stop')\n            self._send_json(self.manager.stop_core())\n            return\n",
            "        if parsed.path == '/api/local_costmap_debug/start':\n            self.manager._trace('HTTP /api/local_costmap_debug/start')\n            result = self.manager.start_local_costmap_debug()\n            self._send_json(result, status=HTTPStatus.OK if result.get('ok') else HTTPStatus.BAD_REQUEST)\n            return\n        if parsed.path == '/api/local_costmap_debug/stop':\n            self.manager._trace('HTTP /api/local_costmap_debug/stop')\n            result = self.manager.stop_local_costmap_debug()\n            self._send_json(result, status=HTTPStatus.OK if result.get('ok') else HTTPStatus.BAD_REQUEST)\n            return\n        if parsed.path == '/api/core/stop':\n            self.manager._trace('HTTP /api/core/stop')\n            self._send_json(self.manager.stop_core())\n            return\n",
            "local costmap debug routes",
        )
    path.write_text(text, encoding="utf-8")


def patch_dashboard_server_floor_testing(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    if "def list_floor_assets(self) -> dict:" not in text:
        text = base.replace_once(
            text,
            "    def start_nvblox(self) -> dict:\n",
            FLOOR_TESTING_METHODS + "    def start_nvblox(self) -> dict:\n",
            "floor testing manager methods",
        )
    if "/api/floors/list" not in text:
        text = base.replace_once(
            text,
            "        if parsed.path == '/api/status':\n            self._send_json(self.manager.status_snapshot())\n            return\n",
            "        if parsed.path == '/api/floors/list':\n            self.manager._trace('HTTP /api/floors/list')\n            self._send_json(self.manager.list_floor_assets())\n            return\n        if parsed.path == '/api/status':\n            self._send_json(self.manager.status_snapshot())\n            return\n",
            "floor testing list route",
        )
    if "/api/floors/promote" not in text:
        text = base.replace_once(
            text,
            "        if parsed.path == '/api/core/stop':\n            self.manager._trace('HTTP /api/core/stop')\n            self._send_json(self.manager.stop_core())\n            return\n",
            "        if parsed.path == '/api/floors/promote':\n            body = self._read_json()\n            self.manager._trace(\n                f'HTTP /api/floors/promote map={body.get(\"map_name\", \"\")} building={body.get(\"building_id\", \"\")} floor={body.get(\"floor_id\", \"\")}'\n            )\n            result = self.manager.promote_map_to_floor(\n                body.get('map_name', ''),\n                body.get('building_id', ''),\n                body.get('floor_id', ''),\n            )\n            self._send_json(result, status=HTTPStatus.OK if result.get('ok') else HTTPStatus.BAD_REQUEST)\n            return\n        if parsed.path == '/api/floors/select':\n            body = self._read_json()\n            self.manager._trace(\n                f'HTTP /api/floors/select building={body.get(\"building_id\", \"\")} floor={body.get(\"floor_id\", \"\")}'\n            )\n            result = self.manager.select_floor_assets(\n                body.get('building_id', ''),\n                body.get('floor_id', ''),\n            )\n            self._send_json(result, status=HTTPStatus.OK if result.get('ok') else HTTPStatus.BAD_REQUEST)\n            return\n        if parsed.path == '/api/floors/switch':\n            body = self._read_json()\n            self.manager._trace(\n                f'HTTP /api/floors/switch building={body.get(\"building_id\", \"\")} floor={body.get(\"floor_id\", \"\")}'\n            )\n            result = self.manager.switch_floor(\n                body.get('building_id', ''),\n                body.get('floor_id', ''),\n            )\n            self._send_json(result, status=HTTPStatus.OK if result.get('ok') else HTTPStatus.BAD_REQUEST)\n            return\n        if parsed.path == '/api/core/stop':\n            self.manager._trace('HTTP /api/core/stop')\n            self._send_json(self.manager.stop_core())\n            return\n",
            "floor testing routes",
        )
    path.write_text(text, encoding="utf-8")


def patch_index_local_costmap_debug(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    if "startLocalCostmapDebugBtn" not in text:
        text = base.replace_once(
            text,
            '          <button class="primary" id="startChassisBtn">启动底盘驱动</button>\n',
            '          <button class="primary" id="startChassisBtn">启动底盘驱动</button>\n          <button class="primary" id="startLocalCostmapDebugBtn">只启动局部障碍地图</button>\n          <button class="danger" id="stopLocalCostmapDebugBtn">停止局部障碍地图</button>\n',
            "local costmap debug buttons",
        )
        text = base.replace_once(
            text,
            "    document.getElementById('stopMappingBtn').onclick = () => runAction(async () => {\n",
            "    document.getElementById('startLocalCostmapDebugBtn').onclick = () => runAction(async () => {\n      const result = await callApi('/api/local_costmap_debug/start', 'POST', null, 90000);\n      openMap2dPopup('', true, false, false, 'local_costmap');\n      alert(result.message || '局部障碍地图已启动');\n      refreshRuntime(true);\n    });\n\n    document.getElementById('stopLocalCostmapDebugBtn').onclick = () => runAction(async () => {\n      const result = await callApi('/api/local_costmap_debug/stop', 'POST', null, 30000);\n      alert(result.message || '局部障碍地图已停止');\n      refreshRuntime(true);\n    });\n\n    document.getElementById('stopMappingBtn').onclick = () => runAction(async () => {\n",
            "local costmap debug button handlers",
        )
    path.write_text(text, encoding="utf-8")


def patch_index_floor_testing(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    if "listFloorAssetsTestBtn" not in text:
        buttons = (
            '          <button id="listFloorAssetsTestBtn">测试：列出楼层资产</button>\n'
            '          <button class="warn" id="promoteFloorAssetTestBtn">测试：归档地图到楼层</button>\n'
            '          <button class="primary" id="selectFloorAssetTestBtn">测试：选择楼层资产</button>\n'
            '          <button class="primary" id="switchFloorAssetTestBtn">测试：切换楼层</button>\n'
        )
        marker = '          <button class="danger" id="stopLocalCostmapDebugBtn">停止局部障碍地图</button>\n'
        if marker in text:
            text = base.replace_once(
                text,
                marker,
                marker + buttons,
                "floor testing buttons after local costmap debug",
            )
        else:
            text = base.replace_once(
                text,
                '          <button class="primary" id="startChassisBtn">启动底盘驱动</button>\n',
                '          <button class="primary" id="startChassisBtn">启动底盘驱动</button>\n' + buttons,
                "floor testing buttons",
            )
        text = base.replace_once(
            text,
            "    document.getElementById('stopMappingBtn').onclick = () => runAction(async () => {\n",
            "    function floorTestSelection() {\n      const defaultBuilding = localStorage.getItem('njrh_test_building_id') || 'building_1';\n      const buildingId = prompt('测试 building_id', defaultBuilding);\n      if (buildingId === null) return null;\n      const defaultFloor = localStorage.getItem('njrh_test_floor_id') || 'floor_1';\n      const floorId = prompt('测试 floor_id', defaultFloor);\n      if (floorId === null) return null;\n      const cleanBuilding = buildingId.trim();\n      const cleanFloor = floorId.trim();\n      if (!cleanBuilding || !cleanFloor) {\n        alert('building_id 和 floor_id 不能为空');\n        return null;\n      }\n      localStorage.setItem('njrh_test_building_id', cleanBuilding);\n      localStorage.setItem('njrh_test_floor_id', cleanFloor);\n      return { building_id: cleanBuilding, floor_id: cleanFloor };\n    }\n\n    function floorTestMapName() {\n      const mapNameInput = document.getElementById('mapNameInput');\n      const savedMap2dInput = document.getElementById('savedMap2dInput');\n      const navMap2dInput = document.getElementById('navMap2dInput');\n      const defaultName = (\n        (mapNameInput && mapNameInput.value.trim()) ||\n        (savedMap2dInput && savedMap2dInput.value.trim()) ||\n        (navMap2dInput && navMap2dInput.value.trim()) ||\n        localStorage.getItem('njrh_test_map_name') ||\n        ''\n      );\n      const mapName = prompt('测试 map_name（来自已保存的 2D/3D 同名地图）', defaultName);\n      if (mapName === null) return null;\n      const cleanName = mapName.trim();\n      if (!cleanName) {\n        alert('map_name 不能为空');\n        return null;\n      }\n      localStorage.setItem('njrh_test_map_name', cleanName);\n      return cleanName;\n    }\n\n    document.getElementById('listFloorAssetsTestBtn').onclick = () => runAction(async () => {\n      const result = await callApi('/api/floors/list', 'GET', null, 30000);\n      const selected = result.selected || {};\n      const floors = Array.isArray(result.floors) ? result.floors : [];\n      const summary = floors.map((floor) => `${floor.valid ? 'OK' : 'MISSING'} ${floor.building_id}/${floor.floor_id}`).join('\\n');\n      alert(`测试楼层资产根目录: ${result.root}\\n当前选择: ${selected.building_id || '-'} / ${selected.floor_id || '-'}\\n\\n${summary || '未发现楼层资产'}`);\n    });\n\n    document.getElementById('promoteFloorAssetTestBtn').onclick = () => runAction(async () => {\n      const mapName = floorTestMapName();\n      if (!mapName) return;\n      const floor = floorTestSelection();\n      if (!floor) return;\n      const result = await callApi('/api/floors/promote', 'POST', { map_name: mapName, ...floor }, 180000);\n      alert(result.message || '测试：楼层资产归档完成');\n      await Promise.allSettled([refreshRuntime(true), loadAssets(true)]);\n    });\n\n    document.getElementById('selectFloorAssetTestBtn').onclick = () => runAction(async () => {\n      const floor = floorTestSelection();\n      if (!floor) return;\n      const result = await callApi('/api/floors/select', 'POST', floor, 60000);\n      alert(result.message || '测试：楼层资产已选择');\n      refreshRuntime(true);\n    });\n\n    document.getElementById('switchFloorAssetTestBtn').onclick = () => runAction(async () => {\n      const floor = floorTestSelection();\n      if (!floor) return;\n      const result = await callApi('/api/floors/switch', 'POST', floor, 150000);\n      alert(result.message || '测试：楼层切换完成');\n      refreshRuntime(true);\n    });\n\n    document.getElementById('stopMappingBtn').onclick = () => runAction(async () => {\n",
            "floor testing button handlers",
        )
    path.write_text(text, encoding="utf-8")


def patch_nvblox_view_runtime_fixed(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    text = base.replace_between(
        text,
        "    let pointsObject = null;\n",
        "\n\n    const robotScene = new THREE.Scene();\n",
        "    let pointsObject = null;\n    let latestCloud = null;\n    let latestRobotPose = null;\n    let latestDisplayLift = { floorZ: 0, offsetZ: 0, enabled: false };\n    let reconnectTimer = null;\n    let socket = null;\n    let hasAutoFit = false;\n    let preserveUserView = false;\n    let applyingProgrammaticView = false;\n\n    function clearPointCloud() {\n      latestCloud = null;\n      latestRobotPose = null;\n      latestDisplayLift = { floorZ: 0, offsetZ: 0, enabled: false };\n      if (pointsObject) {\n        scene.remove(pointsObject);\n        pointsObject.geometry.dispose();\n        pointsObject.material.dispose();\n        pointsObject = null;\n      }\n      if (typeof worldRobotGroup !== 'undefined') {\n        worldRobotGroup.visible = false;\n      }\n      render();\n    }",
        "nvblox declaration block with clearPointCloud",
    )
    text = base.replace_once(
        text,
        "    const robotAxesGroup = new THREE.Group();\n    const robotAxesHelper = new THREE.AxesHelper(1.25);\n    const robotMarker = new THREE.Mesh(\n      new THREE.ConeGeometry(0.16, 0.46, 20),\n      new THREE.MeshStandardMaterial({ color: 0xffb74d, emissive: 0x6a3a00, roughness: 0.35, metalness: 0.1 })\n    );\n    robotMarker.rotation.x = Math.PI * 0.5;\n    robotMarker.position.set(0.22, 0.22, 0);\n    robotAxesGroup.add(robotAxesHelper);\n    robotAxesGroup.add(robotMarker);\n    robotScene.add(robotAxesGroup);\n",
        "    const robotAxesGroup = new THREE.Group();\n    const robotAxesHelper = new THREE.AxesHelper(1.25);\n    const robotMarker = new THREE.Mesh(\n      new THREE.ConeGeometry(0.16, 0.46, 20),\n      new THREE.MeshStandardMaterial({ color: 0xffb74d, emissive: 0x6a3a00, roughness: 0.35, metalness: 0.1 })\n    );\n    robotMarker.rotation.x = Math.PI * 0.5;\n    robotMarker.position.set(0.22, 0.22, 0);\n    robotAxesGroup.add(robotAxesHelper);\n    robotAxesGroup.add(robotMarker);\n    robotScene.add(robotAxesGroup);\n\n    const worldRobotGroup = new THREE.Group();\n    const worldRobotAxes = new THREE.AxesHelper(0.85);\n    const worldRobotMarker = new THREE.Mesh(\n      new THREE.ConeGeometry(0.12, 0.34, 20),\n      new THREE.MeshStandardMaterial({ color: 0x45d7ff, emissive: 0x10384c, roughness: 0.35, metalness: 0.05 })\n    );\n    worldRobotMarker.rotation.x = Math.PI * 0.5;\n    worldRobotMarker.position.set(0.18, 0.16, 0);\n    worldRobotGroup.add(worldRobotAxes);\n    worldRobotGroup.add(worldRobotMarker);\n    worldRobotGroup.visible = false;\n    scene.add(worldRobotGroup);\n",
        "nvblox world robot marker",
    )
    text = base.replace_once(
        text,
        "    function setRobotPose(pose) {\n      if (!pose) {\n        latestRobotPose = null;\n        return;\n      }\n      robotAxesGroup.rotation.set(0, pose.yaw || 0, 0);\n      latestRobotPose = pose;\n    }\n",
        "    function updateWorldRobotPose() {\n      if (!latestRobotPose) {\n        worldRobotGroup.visible = false;\n        return;\n      }\n      const displayLift = latestDisplayLift || { offsetZ: 0 };\n      const poseZ = Number(latestRobotPose.z || 0) + Number(displayLift.offsetZ || 0);\n      const [tx, ty, tz] = rosToThree(\n        Number(latestRobotPose.x || 0),\n        Number(latestRobotPose.y || 0),\n        poseZ\n      );\n      worldRobotGroup.position.set(tx, ty, tz);\n      worldRobotGroup.rotation.set(0, Number(latestRobotPose.yaw || 0), 0);\n      worldRobotGroup.visible = true;\n    }\n\n    function setRobotPose(pose) {\n      if (!pose) {\n        latestRobotPose = null;\n        worldRobotGroup.visible = false;\n        return;\n      }\n      robotAxesGroup.rotation.set(0, pose.yaw || 0, 0);\n      latestRobotPose = pose;\n      updateWorldRobotPose();\n    }\n",
        "nvblox updateWorldRobotPose",
    )
    text = base.replace_between(
        text,
        "      if (!cloud || !cloud.points || !cloud.points.length) {\n",
        "\n\n      const count = cloud.points.length;\n",
        "      if (!cloud || !cloud.points || !cloud.points.length) {\n        clearPointCloud();\n        infoBox.textContent = offlineSavedView\n          ? `等待离线地图 ${savedMapName}...`\n          : '等待 /Laser_map 累计地图数据...';\n        overlayText.textContent = infoBox.textContent;\n        return;\n      }",
        "nvblox empty cloud block",
    )
    text = base.replace_between(
        text,
        "      socket.onmessage = (event) => {\n",
        "\n\n      socket.onclose = () => {\n",
        "      socket.onmessage = (event) => {\n        try {\n          const payload = JSON.parse(event.data);\n          if (!payload.ok) {\n            clearPointCloud();\n            overlayText.textContent = payload.error || '等待 /Laser_map 累计地图数据...';\n            infoBox.textContent = payload.error || '等待 /Laser_map 累计地图数据...';\n            return;\n          }\n          setPointCloud(payload.cloud);\n        } catch (error) {\n          clearPointCloud();\n          overlayText.textContent = String(error);\n        }\n      };",
        "nvblox ws message block",
    )
    text = base.replace_once(
        text,
        "      latestDisplayLift = displayLift;\n",
        "      latestDisplayLift = displayLift;\n      updateWorldRobotPose();\n",
        "nvblox update world robot after display lift",
    )
    text = base.replace_once(
        text,
        "        `display z lift: ${displayLift.enabled ? displayLift.offsetZ.toFixed(3) : '0.000'} (display only)\\n` +\n        `robot inset (/Odometry): ${latestRobotPose ? `${(latestRobotPose.frame_id || cloud.frame_id || 'odom')} (${(latestRobotPose.x || 0).toFixed(2)}, ${(latestRobotPose.y || 0).toFixed(2)}, yaw ${(latestRobotPose.yaw || 0).toFixed(2)})` : 'unavailable'}\\n` +\n",
        "        `display z lift: ${displayLift.enabled ? displayLift.offsetZ.toFixed(3) : '0.000'} (display only)\\n` +\n        `robot main marker: ${latestRobotPose ? `${latestRobotPose.source_frame || 'base_link'} in ${cloud.frame_id || 'odom'}` : 'unavailable'}\\n` +\n        `robot inset (/Odometry): ${latestRobotPose ? `${(latestRobotPose.frame_id || cloud.frame_id || 'odom')} (${(latestRobotPose.x || 0).toFixed(2)}, ${(latestRobotPose.y || 0).toFixed(2)}, yaw ${(latestRobotPose.yaw || 0).toFixed(2)})` : 'unavailable'}\\n` +\n",
        "nvblox robot info block",
    )
    text = base.replace_between(
        text,
        "      socket.onclose = () => {\n",
        "\n\n      socket.onerror = () => {\n",
        "      socket.onclose = () => {\n        clearPointCloud();\n        overlayText.textContent = 'WebSocket 已断开，正在重连...';\n        if (reconnectTimer) {\n          clearTimeout(reconnectTimer);\n        }\n        reconnectTimer = setTimeout(connectWebSocket, 1500);\n      };",
        "nvblox ws close block",
    )
    text = base.replace_between(
        text,
        "      socket.onerror = () => {\n",
        "\n    }\n\n    async function loadOfflineCloud() {\n",
        "      socket.onerror = () => {\n        clearPointCloud();\n        overlayText.textContent = 'WebSocket 连接失败';\n      };",
        "nvblox ws error block",
    )
    path.write_text(text, encoding="utf-8")


def patch_lidar_view_runtime_fixed(path: Path) -> None:
    text = path.read_text(encoding="utf-8")

    text = base.replace_once(
        text,
        "let targetFrame = (searchParams.get('target_frame') || 'base_link').trim();",
        "const DEFAULT_DISPLAY_FRAME = 'base_link';\n    let targetFrame = searchParams.has('target_frame') ? (searchParams.get('target_frame') || '').trim() : DEFAULT_DISPLAY_FRAME;",
        "lidar target frame default",
    )
    text = base.replace_once(
        text,
        "    if (targetFrame.toLowerCase() === 'raw') {\n      targetFrame = '';\n    }\n",
        "    if (targetFrame.toLowerCase() === 'raw') {\n      targetFrame = '';\n    }\n    if (!searchParams.has('target_frame')) {\n      const normalized = new URLSearchParams(searchParams);\n      normalized.set('target_frame', DEFAULT_DISPLAY_FRAME);\n      const nextQuery = normalized.toString();\n      const nextUrl = `${window.location.pathname}${nextQuery ? `?${nextQuery}` : ''}`;\n      window.history.replaceState({}, '', nextUrl);\n    }\n",
        "lidar target frame normalization",
    )
    text = base.replace_once(
        text,
        "    const infoBox = document.getElementById('infoBox');\n    const axisBox = document.getElementById('axisBox');\n    const overlayText = document.getElementById('overlayText');\n    const axisLegend = document.getElementById('axisLegend');\n    const viewer = document.getElementById('viewer');\n    const searchParams = new URLSearchParams(window.location.search);\n",
        "    const infoBox = document.getElementById('infoBox');\n    const axisBox = document.getElementById('axisBox');\n    const overlayText = document.getElementById('overlayText');\n    const axisLegend = document.getElementById('axisLegend');\n    const viewer = document.getElementById('viewer');\n    const rawViewBtn = document.getElementById('rawViewBtn');\n    const baseViewBtn = document.getElementById('baseViewBtn');\n    const searchParams = new URLSearchParams(window.location.search);\n",
        "lidar button capture",
    )
    text = base.replace_once(
        text,
        "    function refreshModeButtons() {\n      const rawBtn = document.getElementById('rawViewBtn');\n      const baseBtn = document.getElementById('baseViewBtn');\n      rawBtn.classList.toggle('mode-active', !targetFrame);\n      baseBtn.classList.toggle('mode-active', targetFrame === 'base_link');\n    }\n",
        "    function refreshModeButtons() {\n      rawViewBtn.classList.toggle('mode-active', !targetFrame);\n      baseViewBtn.classList.toggle('mode-active', targetFrame === 'base_link');\n      rawViewBtn.textContent = 'Raw Sensor Frame (Debug)';\n      baseViewBtn.textContent = 'Vehicle Base Frame base_link';\n    }\n",
        "lidar button labels",
    )
    text = base.replace_once(
        text,
        "scene.add(createRosAxesHelper(1.8));\n\n    const cloudGroup = new THREE.Group();\n    scene.add(cloudGroup);\n",
        "const baseAxesHelper = createRosAxesHelper(1.8);\n    scene.add(baseAxesHelper);\n\n    const sensorPoseGroup = new THREE.Group();\n    const sensorPoseBody = new THREE.Mesh(\n      new THREE.SphereGeometry(0.10, 18, 18),\n      new THREE.MeshBasicMaterial({ color: 0x45d7ff, transparent: true, opacity: 0.95 })\n    );\n    const sensorPoseAxes = createRosAxesHelper(0.75);\n    sensorPoseGroup.add(sensorPoseBody);\n    sensorPoseGroup.add(sensorPoseAxes);\n    sensorPoseGroup.visible = false;\n    scene.add(sensorPoseGroup);\n\n    const cloudGroup = new THREE.Group();\n    scene.add(cloudGroup);\n",
        "lidar sensor pose helpers",
    )
    text = base.replace_once(
        text,
        "    function threeMatrixFromRosMatrix(rosMatrix) {\n      const basis = rosBasisMatrix();\n      const basisInv = basis.clone().invert();\n      return basis.clone().multiply(rosMatrix).multiply(basisInv);\n    }\n\n    function currentAxis() {\n",
        "    function threeMatrixFromRosMatrix(rosMatrix) {\n      const basis = rosBasisMatrix();\n      const basisInv = basis.clone().invert();\n      return basis.clone().multiply(rosMatrix).multiply(basisInv);\n    }\n\n    function updateSensorPoseHelper(cloud) {\n      sensorPoseGroup.position.set(0, 0, 0);\n      sensorPoseGroup.quaternion.identity();\n      sensorPoseGroup.scale.set(1, 1, 1);\n      sensorPoseGroup.visible = false;\n\n      if (!cloud) {\n        return;\n      }\n\n      let sensorPoseRos = new THREE.Matrix4().identity();\n      if (\n        cloud.transform_applied &&\n        cloud.transform &&\n        !cloud.transform.error &&\n        cloud.transform.translation &&\n        cloud.transform.quaternion\n      ) {\n        sensorPoseRos = rosMatrixFromTransform(\n          cloud.transform.translation,\n          cloud.transform.quaternion\n        );\n      }\n\n      const sensorPoseThree = threeMatrixFromRosMatrix(sensorPoseRos);\n      sensorPoseThree.decompose(sensorPoseGroup.position, sensorPoseGroup.quaternion, sensorPoseGroup.scale);\n      sensorPoseGroup.visible = true;\n    }\n\n    function currentAxis() {\n",
        "lidar sensor pose updater",
    )
    text = base.replace_between(
        text,
        "      if (!cloud || !cloud.points || !cloud.points.length) {\n",
        "\n\n      const count = cloud.points.length;\n",
        "      if (!cloud || !cloud.points || !cloud.points.length) {\n        infoBox.textContent = '等待 /lidar_points -> base_link 数据...';\n        overlayText.textContent = infoBox.textContent;\n        sensorPoseGroup.visible = false;\n        axisLegend.innerHTML = '<strong>ROS axes (display frame)</strong><br><span style=\"color:#45d7ff;\">cyan sphere + small axes = lidar_link pose</span>';\n        updateAxisBox();\n        return;\n      }",
        "lidar empty cloud hint",
    )
    text = base.replace_between(
        text,
        "      const displayFrame = cloud.display_frame || cloud.frame_id || 'unknown';\n",
        "\n\n      applyManualAxis();\n",
        "      const displayFrame = cloud.display_frame || cloud.frame_id || 'unknown';\n      overlayText.textContent = `JT128 point cloud ${displayFrame}`;\n      axisLegend.innerHTML = `<strong>ROS axes (${displayFrame})</strong><br><span style=\"color:#45d7ff;\">cyan sphere + small axes = lidar_link pose</span>`;\n      updateSensorPoseHelper(cloud);\n      infoBox.textContent =\n        `source: ${cloud.source || 'unknown'}\\n` +\n        `topic: ${cloud.topic || '/lidar_points'}\\n` +\n        `source frame: ${cloud.source_frame || cloud.frame_id || 'unknown'}\\n` +\n        `display frame: ${displayFrame}\\n` +\n        `tf applied: ${cloud.transform_applied ? 'yes' : 'no'}\\n` +\n        `tf available: ${cloud.transform_available === false ? 'no' : 'yes'}\\n` +\n        `points(sampled): ${cloud.points.length}\\n` +\n        `source points: ${cloud.source_points || cloud.width || cloud.points.length}\\n` +\n        `sample stride: ${cloud.sample_stride || 1}\\n` +\n        `updated: ${new Date((cloud.updated_at || Date.now() / 1000) * 1000).toLocaleTimeString()}\\n` +\n        `${formatTransform(cloud)}\\n` +\n        `sensor marker: cyan sphere = lidar_link origin\\n` +\n        `camera: (${camera.position.x.toFixed(2)}, ${camera.position.y.toFixed(2)}, ${camera.position.z.toFixed(2)})`;",
        "lidar pose legend and sensor marker",
    )
    path.write_text(text, encoding="utf-8")


def patch_lidar_view_rpy_math_fix(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    text = base.replace_between(
        text,
        "    function poseFromRosMatrix(rosMatrix) {\n",
        "\n\n    function updateSensorPoseHelper(cloud) {\n",
        "    function rpyFromQuaternion(quaternion) {\n      const qx = Number(quaternion.x) || 0;\n      const qy = Number(quaternion.y) || 0;\n      const qz = Number(quaternion.z) || 0;\n      const qw = Number(quaternion.w) || 1;\n\n      const sinrCosp = 2 * (qw * qx + qy * qz);\n      const cosrCosp = 1 - 2 * (qx * qx + qy * qy);\n      const roll = Math.atan2(sinrCosp, cosrCosp);\n\n      const sinp = 2 * (qw * qy - qz * qx);\n      const clampedSinp = Math.max(-1, Math.min(1, sinp));\n      const pitch = Math.asin(clampedSinp);\n\n      const sinyCosp = 2 * (qw * qz + qx * qy);\n      const cosyCosp = 1 - 2 * (qy * qy + qz * qz);\n      const yaw = Math.atan2(sinyCosp, cosyCosp);\n\n      return { roll, pitch, yaw };\n    }\n\n    function poseFromRosMatrix(rosMatrix) {\n      const translation = new THREE.Vector3();\n      const quaternion = new THREE.Quaternion();\n      const scale = new THREE.Vector3();\n      rosMatrix.decompose(translation, quaternion, scale);\n      return {\n        translation: { x: translation.x, y: translation.y, z: translation.z },\n        quaternion,\n        rpy: rpyFromQuaternion(quaternion),\n      };\n    }",
        "poseFromRosMatrix ros rpy fix",
    )
    path.write_text(text, encoding="utf-8")


def patch_lidar_view_clear_stale(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    text = base.replace_between(
        text,
        "    let pointsObject = null;\n",
        "\n\n    function resize() {\n",
        "    let pointsObject = null;\n    let latestCloud = null;\n    let reconnectTimer = null;\n    let socket = null;\n    let hasAutoFit = false;\n    let preserveUserView = false;\n    let applyingProgrammaticView = false;\n    let calibrationMode = false;\n    let dragState = null;\n    let axisReferenceAvailable = false;\n    const baseAxis = { roll: 0, pitch: 0, yaw: 0 };\n    const baseMountTranslation = { x: 0, y: 0, z: 0 };\n    const manualDelta = { roll: 0, pitch: 0, yaw: 0 };\n\n    function clearPointCloud() {\n      latestCloud = null;\n      sensorPoseGroup.visible = false;\n      if (pointsObject) {\n        cloudGroup.remove(pointsObject);\n        pointsObject.geometry.dispose();\n        pointsObject.material.dispose();\n        pointsObject = null;\n      }\n      render();\n    }",
        "lidar declaration block with clearPointCloud",
    )
    text = base.replace_between(
        text,
        "      if (!cloud || !cloud.points || !cloud.points.length) {\n",
        "\n\n      const count = cloud.points.length;\n",
        "      if (!cloud || !cloud.points || !cloud.points.length) {\n        clearPointCloud();\n        infoBox.textContent = '等待 /lidar_points -> base_link 数据...';\n        overlayText.textContent = infoBox.textContent;\n        axisLegend.innerHTML = '<strong>ROS axes (display frame)</strong><br><span style=\"color:#45d7ff;\">cyan sphere + small axes = lidar_link pose</span>';\n        updateAxisBox();\n        return;\n      }",
        "lidar empty cloud block",
    )
    text = base.replace_between(
        text,
        "      socket.onmessage = (event) => {\n",
        "\n\n      socket.onclose = () => {\n",
        "      socket.onmessage = (event) => {\n        try {\n          const payload = JSON.parse(event.data);\n          if (!payload.ok) {\n            clearPointCloud();\n            overlayText.textContent = payload.error || '等待 /lidar_points 数据...';\n            infoBox.textContent = payload.error || '等待 /lidar_points 数据...';\n            return;\n          }\n          setPointCloud(payload.cloud);\n        } catch (error) {\n          clearPointCloud();\n          overlayText.textContent = String(error);\n        }\n      };",
        "lidar ws message block",
    )
    text = base.replace_between(
        text,
        "      socket.onclose = () => {\n",
        "\n\n      socket.onerror = () => {\n",
        "      socket.onclose = () => {\n        clearPointCloud();\n        overlayText.textContent = 'WebSocket 已断开，正在重连...';\n        if (reconnectTimer) {\n          clearTimeout(reconnectTimer);\n        }\n        reconnectTimer = setTimeout(connectWebSocket, 1500);\n      };",
        "lidar ws close block",
    )
    text = base.replace_between(
        text,
        "      socket.onerror = () => {\n",
        "\n    }\n\n    document.getElementById('rawViewBtn').onclick = () => {\n",
        "      socket.onerror = () => {\n        clearPointCloud();\n        overlayText.textContent = 'WebSocket 连接失败';\n      };",
        "lidar ws error block",
    )
    path.write_text(text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dashboard-server", required=True)
    parser.add_argument("--index-html", required=True)
    parser.add_argument("--map2d-view-html", required=True)
    parser.add_argument("--lidar-view-html", required=True)
    parser.add_argument("--nvblox-view-html", required=True)
    args = parser.parse_args()

    dashboard_server_path = Path(args.dashboard_server)
    base.patch_dashboard_server(dashboard_server_path)
    patch_dashboard_server_runtime_fixed(dashboard_server_path)
    patch_dashboard_server_local_costmap_debug(dashboard_server_path)
    patch_dashboard_server_floor_testing(dashboard_server_path)
    index_path = Path(args.index_html)
    base.patch_index_html(index_path)
    patch_index_local_costmap_debug(index_path)
    patch_index_floor_testing(index_path)
    base.patch_map2d_view(Path(args.map2d_view_html))
    patch_nvblox_view_runtime_fixed(Path(args.nvblox_view_html))

    lidar_view_path = Path(args.lidar_view_html)
    patch_lidar_view_runtime_fixed(lidar_view_path)
    base.patch_lidar_view(lidar_view_path)
    patch_lidar_view_rpy_math_fix(lidar_view_path)
    patch_lidar_view_clear_stale(lidar_view_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

