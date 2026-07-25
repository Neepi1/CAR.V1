# Orbbec Gemini 336L runtime packages

These ARM64 Debian packages are the validated ROS 2 Humble Orbbec SDK used by
the Jetson runtime image. They are kept in the Docker build context because the
package snapshot is not available from the base image's configured APT sources.
The repository `.dockerignore` intentionally exposes only this vendor directory
to `Dockerfile.car`; do not remove its negated Orbbec package rule.
`njrh_container.sh rebuild-image` verifies `SHA256SUMS` and builds from a
temporary minimal context, so unrelated workspace permissions cannot break the
runtime image build.

For an existing validated navigation image, use
`njrh_container.sh build-orbbec-layer`. It uses
`Dockerfile.orbbec-runtime`, installs only these four packages with `dpkg`,
and deliberately has no APT operation so navigation packages cannot change.

Validated package set:

| Package | Version | SHA-256 |
|---|---|---|
| `ros-humble-diagnostic-updater` | `4.0.7-1jammy.20260605.154445` | `dc7932e16083f4aaa81050b71d2de68032f3e96251b9a91c6721d4e6fb96ad0f` |
| `ros-humble-orbbec-camera` | `2.8.6-1jammy.20260610.141607` | `da0f4e927b3a736727bbb0a244748f157abdc9467694369fadcf4c44d01c85a7` |
| `ros-humble-orbbec-camera-msgs` | `2.8.6-1jammy.20260610.135119` | `8356f0d66bbab20ac446078164a8a45d4851f4731dc5b55709f309e663421062` |
| `ros-humble-orbbec-description` | `2.8.6-1jammy.20260610.135452` | `dde22e90732885d183b571793fa8f0665be9b88a8daa858735018c4cc82106af` |

Do not replace these binaries independently. Upgrade the four-package set
together, rebuild `njrh-car:latest`, and repeat the USB reconnect and depth
stream acceptance tests documented in `docs/orbbec_gemini_336l_container.md`.

The close-range `G336X AMR Default v0.0.5` device preset is runtime data, not a
Debian package. Its verified binary lives under
`scripts/jetson/runtime_overlay/config/orbbec_presets/` so it can be selected
or rolled back without rebuilding the validated driver image. Do not replace
the file without updating its pinned SHA-256 and repeating the depth-scale,
fill-rate, and docking-geometry acceptance tests.
