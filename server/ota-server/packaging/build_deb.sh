#!/bin/bash
# Builds freematics-ota_<version>_all.deb from the scripts in the parent
# directory. Run this ON THE TARGET LINUX SERVER (needs dpkg-deb) - not on
# the Windows dev machine.
#
# Usage: ./build_deb.sh [version]   (default version: 1.0.0)
set -euo pipefail
cd "$(dirname "$0")"

VERSION="${1:-1.0.0}"
PKGROOT="$(mktemp -d)"
trap 'rm -rf "$PKGROOT"' EXIT

# --- filesystem layout -------------------------------------------------
install -d "$PKGROOT/DEBIAN"
install -d "$PKGROOT/opt/freematics-ota"
install -d "$PKGROOT/lib/systemd/system"

install -m 644 ../ota_server.py            "$PKGROOT/opt/freematics-ota/"
install -m 644 ../ota_push_watcher.py       "$PKGROOT/opt/freematics-ota/"
install -m 644 ../registry.example.json     "$PKGROOT/opt/freematics-ota/"
install -m 644 ../README.md                 "$PKGROOT/opt/freematics-ota/"
install -m 644 ../freematics-ota.service      "$PKGROOT/lib/systemd/system/"
install -m 644 ../freematics-ota-push.service "$PKGROOT/lib/systemd/system/"

# --- control ------------------------------------------------------------
cat > "$PKGROOT/DEBIAN/control" <<EOF
Package: freematics-ota
Version: $VERSION
Section: net
Priority: optional
Architecture: all
Depends: python3 (>= 3.7)
Maintainer: (self-hosted, no upstream)
Description: Freematics telelogger pull-OTA server + push-decision watcher
 Serves firmware update checks/downloads to Freematics ONE+ telelogger
 devices (ota_server.py) and optionally decides, per registry.json entry,
 when a device is actually due for an update and tells it to check now
 instead of waiting for its polling interval (ota_push_watcher.py).
 .
 registry.json, cert.pem and key.pem are NOT part of this package - they
 hold per-deployment secrets and are created after install (see README).
EOF

# --- maintainer scripts ---------------------------------------------------
cat > "$PKGROOT/DEBIAN/postinst" <<'EOF'
#!/bin/sh
set -e
if ! getent passwd freematics-ota >/dev/null; then
    adduser --system --group --no-create-home --home /opt/freematics-ota freematics-ota
fi
chown -R freematics-ota:freematics-ota /opt/freematics-ota
systemctl daemon-reload || true
echo "freematics-ota installed. Before starting the services:"
echo "  cd /opt/freematics-ota"
echo "  cp registry.example.json registry.json  # then fill in your device(s)"
echo "  openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -days 3650 -nodes -subj '/CN=freematics-ota'"
echo "  chown freematics-ota:freematics-ota registry.json cert.pem key.pem"
echo "Then: systemctl enable --now freematics-ota freematics-ota-push"
exit 0
EOF
chmod 755 "$PKGROOT/DEBIAN/postinst"

cat > "$PKGROOT/DEBIAN/prerm" <<'EOF'
#!/bin/sh
set -e
systemctl stop freematics-ota freematics-ota-push 2>/dev/null || true
systemctl disable freematics-ota freematics-ota-push 2>/dev/null || true
exit 0
EOF
chmod 755 "$PKGROOT/DEBIAN/prerm"

cat > "$PKGROOT/DEBIAN/postrm" <<'EOF'
#!/bin/sh
set -e
if [ "$1" = "purge" ]; then
    deluser --system freematics-ota 2>/dev/null || true
fi
systemctl daemon-reload || true
exit 0
EOF
chmod 755 "$PKGROOT/DEBIAN/postrm"

OUT="freematics-ota_${VERSION}_all.deb"
dpkg-deb --build --root-owner-group "$PKGROOT" "$OUT"
echo "Built $(pwd)/$OUT"
