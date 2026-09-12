#!/usr/bin/env bash
set -euo pipefail

if [[ ! -r /etc/os-release ]]; then
  echo "ERROR: /etc/os-release not found" >&2
  exit 1
fi

. /etc/os-release
ARCH="$(dpkg --print-architecture)"

if [[ "${ID:-}" != "debian" || "${VERSION_CODENAME:-}" != "bookworm" || "$ARCH" != "arm64" ]]; then
  echo "ERROR: this installer is intended for Debian 12 Bookworm arm64." >&2
  echo "Detected: ID=${ID:-?} CODENAME=${VERSION_CODENAME:-?} ARCH=$ARCH" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y ca-certificates curl

# Remove packages which conflict with Docker CE, if present.
for pkg in docker.io docker-compose docker-doc docker-buildx podman-docker containerd runc; do
  if dpkg -s "$pkg" >/dev/null 2>&1; then
    sudo apt-get remove -y "$pkg"
  fi
done

sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/debian
Suites: ${VERSION_CODENAME}
Components: stable
Architectures: ${ARCH}
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker

if ! getent group docker >/dev/null; then
  sudo groupadd docker
fi
sudo usermod -aG docker "$USER"

echo
sudo docker version --format 'Docker Engine: {{.Server.Version}}'
echo

echo "Docker installed successfully."
echo "IMPORTANT: log out and log back in (or reboot) so membership in the docker group takes effect."
