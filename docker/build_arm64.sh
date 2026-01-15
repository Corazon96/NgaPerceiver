#!/bin/bash
# LingerPerceiver ARM64 Build Script (runs inside Docker container)
set -e

cd /workspace

# Clean build directory if requested
if [ "$CLEAN_BUILD" = "True" ] || [ "$CLEAN_BUILD" = "true" ]; then
    echo "Cleaning build directory..."
    rm -rf build_arm64
fi

mkdir -p build_arm64
cd build_arm64

# CMake configuration (Headless mode)
echo "Configuring CMake..."
cmake .. \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_HEADLESS=ON \
    -DBUILD_GUI=OFF \
    -DBUILD_TESTING=OFF

# Build
echo "Building..."
ninja -j$(nproc)

# Create deployment package
echo "Creating deployment package..."
mkdir -p deploy/lib
mkdir -p deploy/config

# Find and copy executable (it's in Release/ subdirectory due to CMAKE_BUILD_TYPE)
EXECUTABLE_PATH=""
if [ -f "Release/linger_perceiver_service" ]; then
    EXECUTABLE_PATH="Release/linger_perceiver_service"
elif [ -f "linger_perceiver_service" ]; then
    EXECUTABLE_PATH="linger_perceiver_service"
else
    echo "ERROR: Cannot find linger_perceiver_service executable!"
    ls -la
    ls -la Release/ 2>/dev/null || true
    exit 1
fi
echo "Found executable: $EXECUTABLE_PATH"
cp "$EXECUTABLE_PATH" deploy/linger_perceiver_service

# Copy config files
cp -r ../config/* deploy/config/ 2>/dev/null || true
cp /workspace/mid360_config.json deploy/config/ 2>/dev/null || true

# Collect shared library dependencies (excluding system base libraries)
echo "Collecting shared library dependencies..."
ldd "$EXECUTABLE_PATH" | grep "=>" | awk '{print $3}' | sort -u | while read lib; do
    if [ -n "$lib" ] && [ -f "$lib" ]; then
        case "$lib" in
            */libc.so*|*/libm.so*|*/libpthread.so*|*/libdl.so*|*/librt.so*|*/ld-linux*|*/libgcc_s.so*|*/libstdc++.so*)
                echo "  Skip (system): $lib"
                ;;
            *)
                # Get real file (resolve symlinks)
                real_lib=$(readlink -f "$lib")
                if [ -f "$real_lib" ]; then
                    base_name=$(basename "$lib")
                    real_name=$(basename "$real_lib")
                    if [ ! -f "deploy/lib/$real_name" ]; then
                        echo "  Copy: $real_lib"
                        cp "$real_lib" deploy/lib/
                    fi
                    # Create symlink if names differ
                    if [ "$base_name" != "$real_name" ] && [ ! -e "deploy/lib/$base_name" ]; then
                        echo "  Link: $base_name -> $real_name"
                        (cd deploy/lib && ln -sf "$real_name" "$base_name")
                    fi
                fi
                ;;
        esac
    fi
done

# Create launch script
cat > deploy/run.sh << 'RUNSCRIPT'
#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export LD_LIBRARY_PATH="$SCRIPT_DIR/lib:$LD_LIBRARY_PATH"

# Usage: ./run.sh [options]
#   Live mode:   ./run.sh
#   Replay mode: ./run.sh -r <file.lgv>
#   With config: ./run.sh -c <livox_config.json>

exec "$SCRIPT_DIR/linger_perceiver_service" \
    --app-config "$SCRIPT_DIR/config/app_config.json" \
    "$@"
RUNSCRIPT
chmod +x deploy/run.sh

# Copy sample lgv files for testing
echo "Copying sample .lgv files..."
cp /workspace/*.lgv deploy/ 2>/dev/null || true

echo ""
echo "=========================================="
echo "Deployment package created!"
echo "Location: build_arm64/deploy/"
echo ""
echo "Contents:"
ls -la deploy/
echo ""
echo "Libraries collected: $(ls deploy/lib/ 2>/dev/null | wc -l) files"
echo "=========================================="
