#!/bin/bash

# ============================================================================
# macOS Package Builder for PID-Lab
# Creates a .app bundle and .dmg installer for distribution
# ============================================================================

set -e  # Exit on any error

# Configuration
APP_NAME="PID-Lab"
BUNDLE_ID="com.pidlab.app"
VERSION="${VERSION:-0.1.0}"
BINARY_NAME="pid-lab"

# Paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
TARGET_DIR="$PROJECT_ROOT/target/release"
BUILD_DIR="$PROJECT_ROOT/target/macos-package"
APP_DIR="$BUILD_DIR/$APP_NAME.app"
DMG_NAME="${APP_NAME}-${VERSION}-macos"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check for required tools
check_dependencies() {
    log_info "Checking dependencies..."
    
    if ! command -v cargo &> /dev/null; then
        log_error "cargo is not installed. Please install Rust."
        exit 1
    fi
    
    if ! command -v hdiutil &> /dev/null; then
        log_error "hdiutil is not available. This script requires macOS."
        exit 1
    fi
    
    log_info "All dependencies found."
}

# Build the release binary
build_binary() {
    log_info "Building release binary..."
    cd "$PROJECT_ROOT"
    
    # Detect architecture
    ARCH=$(uname -m)
    if [ "$ARCH" = "arm64" ]; then
        TARGET="aarch64-apple-darwin"
    else
        TARGET="x86_64-apple-darwin"
    fi
    
    log_info "Building for target: $TARGET"
    cargo build --release --target "$TARGET"
    
    # Update target dir to include architecture
    TARGET_DIR="$PROJECT_ROOT/target/$TARGET/release"
    
    if [ ! -f "$TARGET_DIR/$BINARY_NAME" ]; then
        log_error "Binary not found at $TARGET_DIR/$BINARY_NAME"
        exit 1
    fi
    
    log_info "Binary built successfully."
}

# Create the .app bundle structure
create_app_bundle() {
    log_info "Creating application bundle..."
    
    # Clean previous build
    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"
    
    # Create .app directory structure
    mkdir -p "$APP_DIR/Contents/MacOS"
    mkdir -p "$APP_DIR/Contents/Resources"
    
    # Copy the binary
    cp "$TARGET_DIR/$BINARY_NAME" "$APP_DIR/Contents/MacOS/$APP_NAME"
    chmod +x "$APP_DIR/Contents/MacOS/$APP_NAME"
    
    # Create Info.plist
    cat > "$APP_DIR/Contents/Info.plist" << EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleDevelopmentRegion</key>
    <string>en</string>
    <key>CFBundleDisplayName</key>
    <string>${APP_NAME}</string>
    <key>CFBundleExecutable</key>
    <string>${APP_NAME}</string>
    <key>CFBundleIconFile</key>
    <string>AppIcon</string>
    <key>CFBundleIdentifier</key>
    <string>${BUNDLE_ID}</string>
    <key>CFBundleInfoDictionaryVersion</key>
    <string>6.0</string>
    <key>CFBundleName</key>
    <string>${APP_NAME}</string>
    <key>CFBundlePackageType</key>
    <string>APPL</string>
    <key>CFBundleShortVersionString</key>
    <string>${VERSION}</string>
    <key>CFBundleVersion</key>
    <string>${VERSION}</string>
    <key>LSMinimumSystemVersion</key>
    <string>10.15</string>
    <key>NSHighResolutionCapable</key>
    <true/>
    <key>NSSupportsAutomaticGraphicsSwitching</key>
    <true/>
    <key>CFBundleDocumentTypes</key>
    <array>
        <dict>
            <key>CFBundleTypeName</key>
            <string>Blackbox Log</string>
            <key>CFBundleTypeExtensions</key>
            <array>
                <string>bbl</string>
                <string>BBL</string>
            </array>
            <key>CFBundleTypeRole</key>
            <string>Viewer</string>
        </dict>
    </array>
</dict>
</plist>
EOF

    log_info "Info.plist created."
    
    # Create icon
    create_icns
    
    log_info "Application bundle created at: $APP_DIR"
}

# Create .icns icon file from PNG
create_icns() {
    log_info "Creating application icon..."
    
    ICON_SOURCE="$PROJECT_ROOT/assets/icon-1024.png"
    ICONSET_DIR="$BUILD_DIR/AppIcon.iconset"
    
    if [ ! -f "$ICON_SOURCE" ]; then
        log_warn "Icon source not found at $ICON_SOURCE. Using default icon."
        return
    fi
    
    mkdir -p "$ICONSET_DIR"
    
    # Generate all required icon sizes
    sips -z 16 16     "$ICON_SOURCE" --out "$ICONSET_DIR/icon_16x16.png" 2>/dev/null
    sips -z 32 32     "$ICON_SOURCE" --out "$ICONSET_DIR/icon_16x16@2x.png" 2>/dev/null
    sips -z 32 32     "$ICON_SOURCE" --out "$ICONSET_DIR/icon_32x32.png" 2>/dev/null
    sips -z 64 64     "$ICON_SOURCE" --out "$ICONSET_DIR/icon_32x32@2x.png" 2>/dev/null
    sips -z 128 128   "$ICON_SOURCE" --out "$ICONSET_DIR/icon_128x128.png" 2>/dev/null
    sips -z 256 256   "$ICON_SOURCE" --out "$ICONSET_DIR/icon_128x128@2x.png" 2>/dev/null
    sips -z 256 256   "$ICON_SOURCE" --out "$ICONSET_DIR/icon_256x256.png" 2>/dev/null
    sips -z 512 512   "$ICON_SOURCE" --out "$ICONSET_DIR/icon_256x256@2x.png" 2>/dev/null
    sips -z 512 512   "$ICON_SOURCE" --out "$ICONSET_DIR/icon_512x512.png" 2>/dev/null
    sips -z 1024 1024 "$ICON_SOURCE" --out "$ICONSET_DIR/icon_512x512@2x.png" 2>/dev/null
    
    # Convert iconset to icns
    iconutil -c icns "$ICONSET_DIR" -o "$APP_DIR/Contents/Resources/AppIcon.icns"
    
    # Cleanup
    rm -rf "$ICONSET_DIR"
    
    log_info "Application icon created."
}

# Create DMG installer
create_dmg() {
    log_info "Creating DMG installer..."
    
    DMG_TEMP="$BUILD_DIR/dmg-temp"
    DMG_PATH="$BUILD_DIR/$DMG_NAME.dmg"
    
    # Remove any existing DMG
    rm -f "$DMG_PATH"
    rm -rf "$DMG_TEMP"
    
    # Create temporary directory for DMG contents
    mkdir -p "$DMG_TEMP"
    
    # Copy app to temp directory
    cp -R "$APP_DIR" "$DMG_TEMP/"
    
    # Create symbolic link to Applications folder
    ln -s /Applications "$DMG_TEMP/Applications"
    
    # Create the DMG
    hdiutil create -volname "$APP_NAME" \
        -srcfolder "$DMG_TEMP" \
        -ov -format UDZO \
        "$DMG_PATH"
    
    # Cleanup
    rm -rf "$DMG_TEMP"
    
    log_info "DMG created at: $DMG_PATH"
}

# Optional: Sign the app (requires Developer ID)
sign_app() {
    if [ -n "$DEVELOPER_ID" ]; then
        log_info "Signing application with Developer ID: $DEVELOPER_ID"
        codesign --force --deep --sign "$DEVELOPER_ID" "$APP_DIR"
        codesign --verify --verbose "$APP_DIR"
        log_info "Application signed successfully."
    else
        log_warn "DEVELOPER_ID not set. Skipping code signing."
        log_warn "Users may need to right-click and select 'Open' on first launch."
    fi
}

# Optional: Notarize the app (requires Apple Developer account)
notarize_app() {
    if [ -n "$APPLE_ID" ] && [ -n "$APPLE_APP_PASSWORD" ] && [ -n "$TEAM_ID" ]; then
        log_info "Notarizing application..."
        
        # Create a ZIP for notarization
        ditto -c -k --keepParent "$APP_DIR" "$BUILD_DIR/$APP_NAME.zip"
        
        xcrun notarytool submit "$BUILD_DIR/$APP_NAME.zip" \
            --apple-id "$APPLE_ID" \
            --password "$APPLE_APP_PASSWORD" \
            --team-id "$TEAM_ID" \
            --wait
        
        # Staple the notarization ticket
        xcrun stapler staple "$APP_DIR"
        
        # Cleanup
        rm -f "$BUILD_DIR/$APP_NAME.zip"
        
        log_info "Application notarized successfully."
    else
        log_warn "Notarization credentials not set. Skipping notarization."
        log_warn "Set APPLE_ID, APPLE_APP_PASSWORD, and TEAM_ID to enable notarization."
    fi
}

# Print summary
print_summary() {
    echo ""
    echo "============================================"
    echo -e "${GREEN}Build Complete!${NC}"
    echo "============================================"
    echo ""
    echo "Generated files:"
    echo "  App Bundle: $APP_DIR"
    echo "  DMG Installer: $BUILD_DIR/$DMG_NAME.dmg"
    echo ""
    echo "To install:"
    echo "  1. Open the DMG file"
    echo "  2. Drag $APP_NAME to the Applications folder"
    echo ""
    if [ -z "$DEVELOPER_ID" ]; then
        echo -e "${YELLOW}Note: The app is not signed.${NC}"
        echo "On first launch, users will need to:"
        echo "  1. Right-click the app"
        echo "  2. Select 'Open'"
        echo "  3. Click 'Open' in the dialog"
        echo ""
    fi
}

# Main execution
main() {
    echo ""
    echo "============================================"
    echo "  macOS Package Builder for $APP_NAME"
    echo "============================================"
    echo ""
    
    check_dependencies
    build_binary
    create_app_bundle
    sign_app
    notarize_app
    create_dmg
    print_summary
}

# Run main function
main "$@"
