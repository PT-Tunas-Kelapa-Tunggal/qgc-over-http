#!/bin/bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_root"

export QGC_SKIP_DMG_LAYOUT=1

echo "=== 1. Clean Staging & Previous Tarballs ==="
rm -rf build/staging QGroundControl-macOS-arm64.tar.gz*

echo "=== 2. Configuring CMake (Release, arm64 only, no GStreamer) ==="
python3 tools/configure.py -B build --release -- \
  -DQGC_ENABLE_GST_VIDEOSTREAMING=OFF \
  -DQGC_MACOS_UNIVERSAL_BUILD=OFF \
  -DQGC_BUILD_TESTING=OFF \
  -DQGC_MACOS_SIGN_WITH_IDENTITY=OFF

echo "=== 3. Building QGroundControl ==="
cmake --build build --target QGroundControl --config Release

echo "=== 4. Staging and Bundling with cmake --install ==="
cmake --install build --config Release

echo "=== 5. Packaging Archive & Generating Checksum ==="
tar -czf QGroundControl-macOS-arm64.tar.gz -C build/staging QGroundControl.app
shasum -a 256 QGroundControl-macOS-arm64.tar.gz > QGroundControl-macOS-arm64.tar.gz.sha256

echo "=== Local Verification Succeeded ==="
ls -lh QGroundControl-macOS-arm64.tar.gz*
cat QGroundControl-macOS-arm64.tar.gz.sha256
