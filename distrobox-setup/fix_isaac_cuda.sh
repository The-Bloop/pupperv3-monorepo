#!/bin/bash
# Repair the 0-byte CUDA driver stubs that distrobox --nvidia leaves behind.
#
# THE BUG (distrobox 1.8.2.4): --nvidia bind-mounts every host file matching *nvidia* plus a
# whitelist of driver libs. It touches the destination first, then mounts over it. The mount
# for libcuda.so does NOT reliably take: across restarts of the SAME container it has landed
# (real 49 MB file, nobody:nogroup) and failed (0-byte stub, root:root). It is flaky, not
# first-boot-only, so this has to run before every Isaac launch.
#
# THE SYMPTOM, which does not look like this at all:
#   dlopen("libcuda.so.1")  -> fine
#   dlopen("libcuda.so")    -> OSError: file too short
#   Isaac logs "NVTT Warning: CUDA driver not found", then SEGFAULTS inside
#   SimulationApp.__init__ -- in a Kit render/material thread, not the main python thread.
#   The py-spy traceback shows MainThread idle in _prepare_ui/_wait_for_viewport, which reads
#   like a bug in whatever script you happen to be running.
#
# The right fix would be `distrobox create --init-hooks`, but distrobox only accepts that at
# create time and recreating pupper-jazzy means reinstalling all of ROS 2 Jazzy.
#
# Deliberately NOT silent: it prints what it repaired and why, because a repair that happens
# invisibly on every run is how a broken environment stays broken.

set -e
set -o pipefail

if [ "$CONTAINER_ID" != "pupper-jazzy" ]; then
    echo "ERROR: run inside the pupper-jazzy distrobox." >&2
    exit 1
fi

LIBDIR=/usr/lib/x86_64-linux-gnu
fixed=0

# `find -empty` rather than a name glob: *cuda*/*nvidia* miss libnvcuvid.so entirely.
for stub in $(find "$LIBDIR" -maxdepth 1 -empty -name "*.so*" 2>/dev/null); do
    base=$(basename "$stub")
    real="$LIBDIR/${base}.1"
    if [ ! -f "$real" ]; then
        echo "WARNING: $base is a 0-byte stub but $real does not exist -- cannot repair." >&2
        echo "         Isaac will likely segfault. Check the host's driver install." >&2
        continue
    fi
    echo "REPAIR: $base was a 0-byte stub (distrobox --nvidia bind-mount failed) -> symlink to ${base}.1"
    sudo rm -f "$stub"
    sudo ln -s "${base}.1" "$stub"
    fixed=$((fixed + 1))
done

if [ "$fixed" -gt 0 ]; then
    sudo ldconfig
    echo "REPAIR: fixed $fixed stub(s) and ran ldconfig."
fi

# Verify rather than assume: this is the exact call that decides whether Isaac lives.
python3 -c "import ctypes; ctypes.CDLL('libcuda.so')" 2>/dev/null || {
    echo "ERROR: dlopen('libcuda.so') still fails after repair. Isaac will segfault." >&2
    ls -la "$LIBDIR"/libcuda.so* >&2
    exit 1
}
[ "$fixed" -eq 0 ] && echo "cuda stubs OK (nothing to repair)"
exit 0
