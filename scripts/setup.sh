#!/bin/bash
# Prepares the C++ dependencies of this package, pinned to the versions it was developed
# against: the vcpkg baseline in vcpkg.json, imgui-platform-kit in ports/, and
# dynamic-neural-field-composer at the commit in scripts/dnfc-ref.txt. Run it once before
# `colcon build`; CMakeLists.txt finds the result in deps/dnfc-install.
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DEPS_DIR="$PROJECT_ROOT/deps"

# ── vcpkg ─────────────────────────────────────────────────────────────────────
if [ -z "$VCPKG_ROOT" ]; then
    export VCPKG_ROOT="$HOME/vcpkg"
    echo "VCPKG_ROOT not set. Installing vcpkg to $VCPKG_ROOT..."
    if [ ! -d "$VCPKG_ROOT" ]; then
        git clone https://github.com/microsoft/vcpkg.git "$VCPKG_ROOT"
    fi
    echo ""
    echo "Add the following line to your shell profile (~/.bashrc) to persist VCPKG_ROOT:"
    echo "  export VCPKG_ROOT=$VCPKG_ROOT"
    echo ""
fi
if [ ! -x "$VCPKG_ROOT/vcpkg" ]; then
    "$VCPKG_ROOT/bootstrap-vcpkg.sh" -disableMetrics
fi

# ── vcpkg baseline ────────────────────────────────────────────────────────────
# vcpkg reads the versions pinned by builtin-baseline from the files it has checked out,
# so VCPKG_ROOT has to be a full clone at that commit or a newer one.
BASELINE=$(sed -n 's/.*"builtin-baseline": *"\([0-9a-f]*\)".*/\1/p' "$PROJECT_ROOT/vcpkg.json")
if [ -z "$BASELINE" ]; then
    echo "ERROR: no builtin-baseline found in $PROJECT_ROOT/vcpkg.json."
    exit 1
fi
if [ "$(git -C "$VCPKG_ROOT" rev-parse --is-shallow-repository 2>/dev/null)" = "true" ]; then
    echo "vcpkg at $VCPKG_ROOT is a shallow clone, fetching its full history..."
    git -C "$VCPKG_ROOT" fetch --unshallow origin || {
        echo "ERROR: could not fetch the history of $VCPKG_ROOT. VCPKG_ROOT must be a clone you can write to."
        exit 1
    }
fi
if ! git -C "$VCPKG_ROOT" merge-base --is-ancestor "$BASELINE" HEAD 2>/dev/null; then
    echo "ERROR: vcpkg at $VCPKG_ROOT is older than the pinned baseline $BASELINE, or is not a git clone."
    echo "VCPKG_ROOT must be a git clone of https://github.com/microsoft/vcpkg at that commit or newer."
    echo "To move it to the pinned baseline:"
    echo "  git -C \"$VCPKG_ROOT\" fetch origin && git -C \"$VCPKG_ROOT\" checkout $BASELINE && \"$VCPKG_ROOT/bootstrap-vcpkg.sh\" -disableMetrics"
    exit 1
fi

# ── dynamic-neural-field-composer ─────────────────────────────────────────────
# Built against this package's vcpkg.json (VCPKG_MANIFEST_DIR), so the library and this
# package link the same dependency versions. The install is stamped with the ref and a
# hash of the manifest, so changing either one rebuilds it.
DNFC_SRC="$DEPS_DIR/dynamic-neural-field-composer"
DNFC_INSTALL="$DEPS_DIR/dnfc-install"
DNFC_REF="$(cat "$SCRIPT_DIR/dnfc-ref.txt")"
MANIFEST_HASH=$(cat "$PROJECT_ROOT/vcpkg.json" "$PROJECT_ROOT/vcpkg-configuration.json" "$PROJECT_ROOT"/ports/*/* | git hash-object --stdin)
DNFC_STAMP="$DNFC_REF $MANIFEST_HASH"

mkdir -p "$DEPS_DIR"
# deps/ sits inside the package source; keep colcon from mistaking the clone for a package.
touch "$DEPS_DIR/COLCON_IGNORE"

if [ -d "$DNFC_INSTALL" ] && [ "$(cat "$DNFC_INSTALL/.revision" 2>/dev/null)" != "$DNFC_STAMP" ]; then
    echo "dnfc-install was built from a different revision or dependency set, rebuilding."
    rm -rf "$DNFC_INSTALL"
fi

if [ ! -d "$DNFC_INSTALL" ]; then
    if [ ! -d "$DNFC_SRC" ]; then
        git clone https://github.com/Jgocunha/dynamic-neural-field-composer.git "$DNFC_SRC"
    fi
    if ! git -C "$DNFC_SRC" cat-file -e "$DNFC_REF^{commit}" 2>/dev/null; then
        git -C "$DNFC_SRC" fetch origin
    fi
    git -C "$DNFC_SRC" checkout "$DNFC_REF"

    echo "Building dynamic-neural-field-composer..."
    rm -rf "$DNFC_SRC/build"
    cmake -S "$DNFC_SRC/dynamic-neural-field-composer" -B "$DNFC_SRC/build" \
        -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" \
        -DVCPKG_MANIFEST_DIR="$PROJECT_ROOT" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$DNFC_INSTALL"
    cmake --build "$DNFC_SRC/build" --parallel "$(nproc)"
    cmake --install "$DNFC_SRC/build"
    echo "$DNFC_STAMP" > "$DNFC_INSTALL/.revision"
else
    echo "dynamic-neural-field-composer already installed, skipping."
fi

echo ""
echo "Setup complete. Build the package with colcon as usual."
