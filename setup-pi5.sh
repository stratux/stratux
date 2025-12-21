#!/bin/bash
#
# Stratux AIS - Installation automatique pour Raspberry Pi 5
# Compatible avec Raspberry Pi OS Lite 64-bit (Bookworm/Trixie)
#
# Usage:
#   sudo bash -c "$(wget -nv -O - https://raw.githubusercontent.com/fredoos/stratux_ais/claude/implement-feature-mjej6af2ml9zt0oa-Z7irF/setup-pi5.sh)"
#
# Matériel supporté:
#   - GPS u-blox (USB)
#   - RTL-SDR pour ADS-B (USB)
#   - Waveshare Sense HAT (B) - IMU + Baromètre LPS22HB (GPIO I2C)
#   - Daisy HAT / Récepteur AIS externe (GPIO UART)
#

set -e

# Couleurs pour l'affichage
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

STRATUX_HOME="/opt/stratux"
REPO_URL="https://github.com/fredoos/stratux_ais.git"
BRANCH="claude/implement-feature-mjej6af2ml9zt0oa-Z7irF"

echo -e "${BLUE}"
echo "╔════════════════════════════════════════════════════════════╗"
echo "║           Stratux AIS - Installation Pi 5                  ║"
echo "║                                                            ║"
echo "║  Support: ADS-B, AIS (SDR + External), GPS, AHRS           ║"
echo "║  Capteurs: BMP280/388, LPS22HB, ICM-20948, MPU-9250        ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Vérifier si root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Erreur: Ce script doit être exécuté en root (sudo)${NC}"
    exit 1
fi

# Détecter l'architecture
ARCH=$(uname -m)
if [ "$ARCH" != "aarch64" ]; then
    echo -e "${YELLOW}Attention: Architecture $ARCH détectée. Ce script est optimisé pour arm64/aarch64.${NC}"
fi

# Détecter la version Debian
if [ -f /etc/os-release ]; then
    . /etc/os-release
    echo -e "${GREEN}Système détecté: $PRETTY_NAME${NC}"
fi

echo ""
echo -e "${YELLOW}Installation des dépendances...${NC}"

# Mise à jour système
apt-get update

# Installer les dépendances de base
apt-get install -y \
    git \
    build-essential \
    pkg-config \
    cmake \
    libusb-1.0-0-dev \
    libncurses-dev \
    i2c-tools \
    hostapd \
    dnsmasq \
    iw \
    wireless-tools \
    wget \
    curl

# Installer RTL-SDR (si disponible via apt)
apt-get install -y rtl-sdr librtlsdr-dev librtlsdr2 2>/dev/null || {
    echo -e "${YELLOW}RTL-SDR non disponible via apt, compilation depuis les sources...${NC}"

    cd /tmp
    rm -rf rtl-sdr
    git clone https://github.com/osmocom/rtl-sdr.git
    cd rtl-sdr
    mkdir -p build && cd build
    cmake ../ -DINSTALL_UDEV_RULES=ON -DDETACH_KERNEL_DRIVER=ON
    make -j$(nproc)
    make install
    ldconfig
    cd /tmp
    rm -rf rtl-sdr
}

# Blacklister le driver DVB pour RTL-SDR
echo "blacklist dvb_usb_rtl28xxu" > /etc/modprobe.d/rtl-sdr-blacklist.conf

echo ""
echo -e "${YELLOW}Installation de Go...${NC}"

# Installer Go 1.23.4
GO_VERSION="1.23.4"
GO_TARBALL="go${GO_VERSION}.linux-arm64.tar.gz"

cd /tmp
if [ ! -f "$GO_TARBALL" ]; then
    wget -q "https://go.dev/dl/${GO_TARBALL}"
fi
rm -rf /usr/local/go
tar -C /usr/local -xzf "$GO_TARBALL"
rm -f "$GO_TARBALL"

# Configurer PATH pour Go
export PATH=$PATH:/usr/local/go/bin
export GOPATH=/root/go

# Ajouter au profil
if ! grep -q "/usr/local/go/bin" /etc/profile; then
    echo 'export PATH=$PATH:/usr/local/go/bin' >> /etc/profile
    echo 'export GOPATH=$HOME/go' >> /etc/profile
fi

echo -e "${GREEN}Go $(go version) installé${NC}"

echo ""
echo -e "${YELLOW}Clonage et compilation de Stratux...${NC}"

# Cloner le dépôt
cd /root
rm -rf stratux
git clone "$REPO_URL" stratux
cd stratux
git fetch origin "$BRANCH"
git checkout "$BRANCH"
git submodule update --init --recursive

# Patch dump1090 pour GCC 14+ (corrige l'ordre des arguments calloc)
echo -e "${YELLOW}Application des patches de compatibilité...${NC}"
sed -i 's/calloc(sizeof(\*service), 1)/calloc(1, sizeof(*service))/' dump1090/net_io.c
# Désactiver -Werror si d'autres erreurs similaires apparaissent
sed -i 's/-Werror/-Wno-error=calloc-transposed-args/' dump1090/Makefile

# Compiler
echo -e "${YELLOW}Compilation en cours (peut prendre plusieurs minutes)...${NC}"
make clean 2>/dev/null || true
make all

# Installation manuelle (plus fiable que dpkg sur certaines configurations)
echo -e "${YELLOW}Installation de Stratux...${NC}"

# Créer les répertoires
mkdir -p /opt/stratux/bin
mkdir -p /opt/stratux/cfg
mkdir -p /var/log/stratux

# Copier les binaires
cp -f stratuxrun /opt/stratux/bin/
cp -f fancontrol /opt/stratux/bin/ 2>/dev/null || true
cp -f dump1090/dump1090 /opt/stratux/bin/
cp -f libdump978.so /opt/stratux/bin/ 2>/dev/null || true
cp -f rtl-ais/rtl_ais /opt/stratux/bin/ 2>/dev/null || true
cp -f ogn/ogn-* /opt/stratux/bin/ 2>/dev/null || true

# Copier l'interface web
cp -rf web /opt/stratux/

# Créer le service systemd
cat > /etc/systemd/system/stratux.service << 'EOF'
[Unit]
Description=Stratux ADS-B/AIS Receiver
After=network.target

[Service]
Type=simple
ExecStart=/opt/stratux/bin/stratuxrun
WorkingDirectory=/opt/stratux
Restart=always
RestartSec=3
Environment=LD_LIBRARY_PATH=/opt/stratux/bin

[Install]
WantedBy=multi-user.target
EOF

echo -e "${GREEN}Stratux installé dans /opt/stratux${NC}"

echo ""
echo -e "${YELLOW}Configuration du système...${NC}"

# Activer I2C
if command -v raspi-config &> /dev/null; then
    raspi-config nonint do_i2c 0
    raspi-config nonint do_serial_hw 0
    raspi-config nonint do_serial_cons 1
fi

# Configuration /boot/firmware/config.txt
CONFIG_FILE="/boot/firmware/config.txt"
if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE="/boot/config.txt"
fi

if [ -f "$CONFIG_FILE" ]; then
    # Ajouter configurations si pas déjà présentes
    grep -q "dtparam=i2c_arm=on" "$CONFIG_FILE" || echo "dtparam=i2c_arm=on" >> "$CONFIG_FILE"
    grep -q "enable_uart=1" "$CONFIG_FILE" || echo "enable_uart=1" >> "$CONFIG_FILE"
    grep -q "dtoverlay=uart0" "$CONFIG_FILE" || echo "dtoverlay=uart0" >> "$CONFIG_FILE"

    echo -e "${GREEN}Configuration boot mise à jour${NC}"
fi

# Débloquer le WiFi (rfkill)
echo -e "${YELLOW}Déblocage WiFi...${NC}"
rfkill unblock wifi 2>/dev/null || true

# Configuration hostapd (WiFi Access Point)
cat > /etc/hostapd/hostapd.conf << 'EOF'
interface=wlan0
driver=nl80211
ssid=stratux
hw_mode=g
channel=1
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=0
EOF

echo 'DAEMON_CONF="/etc/hostapd/hostapd.conf"' > /etc/default/hostapd

# Configuration dnsmasq
cat > /etc/dnsmasq.d/stratux.conf << 'EOF'
interface=wlan0
dhcp-range=192.168.10.10,192.168.10.50,255.255.255.0,12h
address=/stratux.local/192.168.10.1
EOF

# Désactiver NetworkManager pour wlan0 (crucial pour Pi OS Bookworm+)
echo -e "${YELLOW}Configuration NetworkManager pour libérer wlan0...${NC}"

mkdir -p /etc/NetworkManager/conf.d
cat > /etc/NetworkManager/conf.d/stratux.conf << 'EOF'
[keyfile]
unmanaged-devices=interface-name:wlan0
EOF

# Supprimer les connexions WiFi existantes sur wlan0
if command -v nmcli &> /dev/null; then
    nmcli device set wlan0 managed no 2>/dev/null || true
    # Supprimer toutes les connexions WiFi
    for conn in $(nmcli -t -f NAME,TYPE connection show | grep wireless | cut -d: -f1); do
        nmcli connection delete "$conn" 2>/dev/null || true
    done
fi

# Configuration réseau wlan0 (IP statique)
mkdir -p /etc/network/interfaces.d
cat > /etc/network/interfaces.d/wlan0 << 'EOF'
auto wlan0
iface wlan0 inet static
    address 192.168.10.1
    netmask 255.255.255.0
EOF

# Créer un service pour configurer wlan0 avant hostapd
cat > /etc/systemd/system/stratux-wifi.service << 'EOF'
[Unit]
Description=Stratux WiFi AP Setup
Before=hostapd.service
After=network-pre.target
Wants=network-pre.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/sh -c 'ip link set wlan0 down; ip addr flush dev wlan0; ip addr add 192.168.10.1/24 dev wlan0; ip link set wlan0 up'

[Install]
WantedBy=multi-user.target
EOF

# Créer le fichier de configuration par défaut
mkdir -p /opt/stratux/cfg
cat > /opt/stratux/cfg/stratux.conf << 'EOF'
{
    "UAT_Enabled": false,
    "ES_Enabled": true,
    "OGN_Enabled": false,
    "AIS_Enabled": true,
    "ExternalAIS_Enabled": false,
    "ExternalAIS_SerialPort": "/dev/ttyAMA0",
    "ExternalAIS_BaudRate": 38400,
    "GPS_Enabled": true,
    "IMU_Sensor_Enabled": true,
    "BMP_Sensor_Enabled": true,
    "DeveloperMode": false
}
EOF

echo ""
echo -e "${YELLOW}Configuration des services...${NC}"

# Activer les services
systemctl unmask hostapd 2>/dev/null || true
systemctl daemon-reload
systemctl enable stratux-wifi
systemctl enable hostapd
systemctl enable dnsmasq
systemctl enable stratux

# Désactiver services qui interfèrent avec le WiFi AP
systemctl stop wpa_supplicant 2>/dev/null || true
systemctl disable wpa_supplicant 2>/dev/null || true
systemctl mask wpa_supplicant 2>/dev/null || true

# Désactiver services inutiles
systemctl disable apt-daily.timer 2>/dev/null || true
systemctl disable apt-daily-upgrade.timer 2>/dev/null || true
systemctl disable man-db.timer 2>/dev/null || true

# Redémarrer NetworkManager pour appliquer les changements
systemctl restart NetworkManager 2>/dev/null || true

echo ""
echo -e "${GREEN}"
echo "╔════════════════════════════════════════════════════════════╗"
echo "║              Installation terminée!                        ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""
echo "Configuration matérielle détectée:"
echo "-----------------------------------"

# Vérifier I2C
if command -v i2cdetect &> /dev/null; then
    echo -e "${BLUE}Capteurs I2C:${NC}"
    i2cdetect -y 1 2>/dev/null | grep -E "^[0-9]" || echo "  (aucun détecté - redémarrez d'abord)"
fi

# Vérifier ports série
echo -e "${BLUE}Ports série:${NC}"
ls /dev/ttyUSB* /dev/ttyACM* /dev/ttyAMA* /dev/serial* 2>/dev/null || echo "  (aucun détecté)"

# Vérifier RTL-SDR
echo -e "${BLUE}RTL-SDR:${NC}"
lsusb | grep -i rtl || echo "  (aucun détecté)"

echo ""
echo -e "${YELLOW}Prochaines étapes:${NC}"
echo "1. Redémarrez: sudo reboot"
echo "2. Connectez-vous au WiFi 'stratux'"
echo "3. Ouvrez http://192.168.10.1"
echo ""
echo -e "${YELLOW}Configuration AIS externe (Daisy HAT):${NC}"
echo "  Settings → External AIS → Enabled"
echo "  Serial Port: /dev/ttyAMA0"
echo "  Baud Rate: 38400"
echo ""

read -p "Redémarrer maintenant? [o/N] " -n 1 -r
echo
if [[ $REPLY =~ ^[Oo]$ ]]; then
    reboot
fi
