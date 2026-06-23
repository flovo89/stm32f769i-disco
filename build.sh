#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEST="$SCRIPT_DIR/venv/bin/west"
WORKSPACE="$SCRIPT_DIR/zephyrproject"
BUILD_DIR="$SCRIPT_DIR/build"
BOARD="stm32f769i_disco"

usage() {
    echo "Usage: $0 [build|flash|clean|rebuild]"
    echo "  build    (default) Build the firmware"
    echo "  flash    Flash to board via west runner"
    echo "  clean    Remove build directory"
    echo "  rebuild  Clean then build"
    exit 1
}

cmd="${1:-build}"

case "$cmd" in
    clean)
        echo "Cleaning $BUILD_DIR ..."
        rm -rf "$BUILD_DIR"
        ;;
    build)
        cd "$WORKSPACE"
        "$WEST" build -b "$BOARD" "$SCRIPT_DIR" --build-dir "$BUILD_DIR"
        ;;
    flash)
        # ST-Link USB permissions — look up device node via sysfs (stable across re-enumeration)
        for vendor_file in /sys/bus/usb/devices/*/idVendor; do
            [[ -f "$vendor_file" ]] || continue
            grep -q "0483" "$vendor_file" || continue
            dir=$(dirname "$vendor_file")
            grep -q "^374" "$dir/idProduct" 2>/dev/null || continue
            busnum=$(cat "$dir/busnum")
            devnum=$(cat "$dir/devnum")
            node=$(printf "/dev/bus/usb/%03d/%03d" "$busnum" "$devnum")
            if [[ -e "$node" && ! -w "$node" ]]; then
                echo "Fixing permissions on $node (requires sudo once) ..."
                sudo chmod a+rw "$node"
            fi
        done
        cd "$WORKSPACE"
        "$WEST" flash --build-dir "$BUILD_DIR" --runner openocd
        ;;
    rebuild)
        rm -rf "$BUILD_DIR"
        cd "$WORKSPACE"
        "$WEST" build -b "$BOARD" "$SCRIPT_DIR" --build-dir "$BUILD_DIR"
        ;;
    *)
        usage
        ;;
esac
