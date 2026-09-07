# Maps

- `maps_feature_module.{hpp,cpp}` is the aggregate owner. It owns
  `MapRuntimeStateStore`, the cross-asset commit mutex, and `MapsModule`, and
  binds late neighboring runtime observations without changing their policy.
  The API root owns only this aggregate, not the catalog/store module or its
  port graph.
- `maps_module.{hpp,cpp}` is the API-facing deep module. It owns catalog
  startup migration/recovery, activation journals and integrity latches, all
  `/api/v1/maps...` HTTP routes, pose mutation, keepout ROS clients and
  subscriptions, and runtime keepout effectiveness proof. The aggregate
  supplies only runtime snapshots, stationary-pose evidence, elevator
  reference lookup, floor interlock, and motion-admission ports.
- `maps_configuration_module.{hpp,cpp}` is the single declaration boundary
  for the 15 maps, runtime-path, and keepout ROS parameters. It preserves the
  existing defaults and timeout clamp, emits the complete `MapsModuleConfig`,
  and exposes one immutable `MapsRuntimePaths` projection to neighboring
  modules. It performs no filesystem mutation, map switch, or ROS call.
- `catalog_activation/` owns immutable map identity, catalog lookup, activation
  filesystem safety, manifests, previews, shared map models, and the small
  runtime-context/last-selection persistence adapter used by neighboring
  modules.
- `poses/` owns semantic point persistence.
- `keepout/` owns editable keepout semantics and raster transactions.

Map saving belongs to the sibling `features/mapping/` directory. Floor-switch
transaction orchestration belongs to `features/floor_switch/`. The maps module
does not start navigation, switch floors, or alter TF/DDS/Nav2 parameters.
