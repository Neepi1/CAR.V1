# Map catalog and activation

Owns the immutable map catalog and the filesystem rules needed to read,
authenticate, activate, project, preview, and delete exact map bundles. This
includes map models, manifests, asset digests, safe IDs, bounded file IO, and
runtime preview lookup.

It does not call ROS services, trigger localization, start Nav2, choose an App
workflow, or perform a live floor switch. Transaction orchestration remains in
the corresponding feature/application layer.
