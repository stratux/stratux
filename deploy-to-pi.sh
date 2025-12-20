#!/bin/bash
#
# Script de déploiement Stratux vers Raspberry Pi
# Usage: ./deploy-to-pi.sh [adresse_ip]
#

PI_HOST="${1:-192.168.10.1}"
PI_USER="pi"
STRATUX_HOME="/opt/stratux"

echo "=== Déploiement Stratux vers $PI_HOST ==="

# Vérifier la connexion
echo "[1/5] Vérification de la connexion..."
if ! ssh -o ConnectTimeout=5 ${PI_USER}@${PI_HOST} "echo OK" > /dev/null 2>&1; then
    echo "ERREUR: Impossible de se connecter à ${PI_HOST}"
    echo "Vérifiez que vous êtes connecté au WiFi Stratux"
    exit 1
fi

# Arrêter Stratux
echo "[2/5] Arrêt de Stratux..."
ssh ${PI_USER}@${PI_HOST} "sudo systemctl stop stratux"

# Transférer les fichiers web
echo "[3/5] Transfert des fichiers web..."
scp -r web/* ${PI_USER}@${PI_HOST}:/tmp/stratux-web/
ssh ${PI_USER}@${PI_HOST} "sudo cp -r /tmp/stratux-web/* ${STRATUX_HOME}/www/"

# Transférer le binaire (si compilé pour ARM)
if [ -f "stratuxrun-arm" ]; then
    echo "[4/5] Transfert du binaire..."
    scp stratuxrun-arm ${PI_USER}@${PI_HOST}:/tmp/
    ssh ${PI_USER}@${PI_HOST} "sudo cp /tmp/stratuxrun-arm ${STRATUX_HOME}/bin/stratuxrun && sudo chmod +x ${STRATUX_HOME}/bin/stratuxrun"
else
    echo "[4/5] Pas de binaire ARM trouvé (stratuxrun-arm), transfert du code source..."
    # Créer archive et transférer pour compilation sur Pi
    tar czvf /tmp/stratux-src.tar.gz --exclude='.git' --exclude='*.tar.gz' .
    scp /tmp/stratux-src.tar.gz ${PI_USER}@${PI_HOST}:/tmp/

    echo "    Compilation sur le Raspberry Pi (cela peut prendre quelques minutes)..."
    ssh ${PI_USER}@${PI_HOST} << 'ENDSSH'
        cd /tmp
        rm -rf stratux-build
        mkdir stratux-build
        cd stratux-build
        tar xzf ../stratux-src.tar.gz

        # Compiler uniquement le binaire principal
        export STRATUX_HOME=/opt/stratux
        go build -ldflags "-X main.stratuxVersion=dev -X main.stratuxBuild=$(date +%Y%m%d)" -o stratuxrun ./main/

        # Installer
        sudo cp stratuxrun /opt/stratux/bin/
        sudo chmod +x /opt/stratux/bin/stratuxrun
ENDSSH
fi

# Redémarrer Stratux
echo "[5/5] Redémarrage de Stratux..."
ssh ${PI_USER}@${PI_HOST} "sudo systemctl start stratux"

echo ""
echo "=== Déploiement terminé ==="
echo "Accédez à http://${PI_HOST} pour vérifier"
echo ""
echo "Pour voir les logs:"
echo "  ssh ${PI_USER}@${PI_HOST} 'journalctl -u stratux -f'"
