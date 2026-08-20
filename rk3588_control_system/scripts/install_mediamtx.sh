#!/bin/bash

set -euo pipefail

VERSION="1.19.2"
ARCHIVE_NAME="mediamtx_v${VERSION}_linux_arm64.tar.gz"
EXPECTED_SHA256="562f419912a8668c18216a9e8c95359ec82fbb754e4a44e2953ef62b98eec688"
ARCHIVE_PATH="${1:-/tmp/${ARCHIVE_NAME}}"
DOWNLOAD_URL="https://github.com/bluenviron/mediamtx/releases/download/v${VERSION}/${ARCHIVE_NAME}"

if [[ $EUID -ne 0 ]]; then
    echo "[mediamtx] Please run with sudo." >&2
    exit 1
fi

if [[ "$(uname -m)" != "aarch64" ]]; then
    echo "[mediamtx] This installer is pinned to Linux arm64/aarch64." >&2
    exit 1
fi

if [[ ! -f "$ARCHIVE_PATH" ]]; then
    echo "[mediamtx] Downloading ${DOWNLOAD_URL}"
    curl --fail --location --retry 3 --output "$ARCHIVE_PATH" "$DOWNLOAD_URL"
fi

echo "${EXPECTED_SHA256}  ${ARCHIVE_PATH}" | sha256sum --check --status
temp_dir="$(mktemp -d)"
trap 'rm -rf "$temp_dir"' EXIT
tar -xzf "$ARCHIVE_PATH" -C "$temp_dir" mediamtx LICENSE
install -m 0755 "$temp_dir/mediamtx" /usr/local/bin/mediamtx
install -m 0644 "$temp_dir/LICENSE" /usr/local/share/doc/mediamtx-LICENSE
/usr/local/bin/mediamtx --version

