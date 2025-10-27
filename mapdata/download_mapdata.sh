#!/bin/bash
set -euo pipefail

# -------------------- CONFIGURATION --------------------

ofm_desc="OpenFlightMaps (AIRAC 2510)"

declare -A region_urls=(
  ["OpenFlightMaps VFR Charts Europe (~700 MiB)"]="https://abatzill.de/stratux/openflightmaps.mbtiles"
  ["US Sectional VFR Charts (~4.9 GiB)"]="https://abatzill.de/stratux/vfrsec.mbtiles"
  ["EB Belgium, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/ebbu/noninteractive/epsg3857/ebbu_256.mbtiles"
  ["ED Germany, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/ed/noninteractive/epsg3857/ed_256.mbtiles"
  ["EF Finland, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/efin/noninteractive/epsg3857/efin_256.mbtiles"
  ["EH Netherlands, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/ehaa/noninteractive/epsg3857/ehaa_256.mbtiles"
  ["EK Denmark, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/ekdk/noninteractive/epsg3857/ekdk_256.mbtiles"
  ["EP Poland, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/epww/noninteractive/epsg3857/epww_256.mbtiles"
  ["ES Sweden, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/esaa/noninteractive/epsg3857/esaa_256.mbtiles"
  ["LB Bulgaria, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/lbsr/noninteractive/epsg3857/lbsr_256.mbtiles"
  ["LD Croatia, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/ldzo/noninteractive/epsg3857/ldzo_256.mbtiles"
  ["LG Greece, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/lggg/noninteractive/epsg3857/lggg_256.mbtiles"
  ["LH Hungary, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/lhcc/noninteractive/epsg3857/lhcc_256.mbtiles"
  ["LI Italy, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/li/noninteractive/epsg3857/li_256.mbtiles"
  ["LJ Slovenia, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/ljla/noninteractive/epsg3857/ljla_256.mbtiles"
  ["LK Czech Republic, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/lkaa/noninteractive/epsg3857/lkaa_256.mbtiles"
  ["LO Austria, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/lovv/noninteractive/epsg3857/lovv_256@2x.mbtiles"
  ["LR Romania, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/lrbb/noninteractive/epsg3857/lrbb_256.mbtiles"
  ["LS Switzerland, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/lsas/noninteractive/epsg3857/lsas_256.mbtiles"
  ["LZ Slovakia, $ofm_desc"]="https://snapshots.openflightmaps.org/live/2510/tiles/lzbb/noninteractive/epsg3857/lzbb_256.mbtiles"
)

# ---------------------- LOGIC ----------------------

cd /
sudo overlayctl unlock
cd /overlay/robase/opt/stratux/mapdata

echo "Waiting for time synchronization..."
sudo systemctl start systemd-timesyncd
while [ "$(timedatectl show | grep NTPSync | grep yes)" == "" ]; do
    echo -n "."
    sleep 1
done
sudo systemctl stop systemd-timesyncd

# Global variables
tmpfile=""
CLEANUP_DONE=0

cleanup() {
    (( CLEANUP_DONE )) && return
    CLEANUP_DONE=1

    echo
    echo "Cleaning up..."
    if [[ -n "$tmpfile" && -f "$tmpfile" ]]; then
        echo "Removing temporary file: $tmpfile"
        rm -f "$tmpfile" || echo "Warning: Could not remove $tmpfile"
    fi

    echo "Restarting stratux..."
    cd /
    sync
    sudo overlayctl lock
    sudo systemctl daemon-reload
    sudo systemctl restart stratux
}

trap cleanup EXIT
trap cleanup INT TERM

IFS=$'\n' read -r -d '' -a regions < <(printf '%s\n' "${!region_urls[@]}" | sort && printf '\0')

echo "Select a region to download (or press 'e' to exit):"
for i in "${!regions[@]}"; do
  printf "%2d) %s\n" "$((i+1))" "${regions[$i]}"
done

read -rp "#? " choice

if [[ "$choice" =~ ^[eE]$ ]]; then
  echo "Exiting..."
  exit 0
fi

if [[ "$choice" =~ ^[0-9]+$ ]] && (( choice >= 1 && choice <= ${#regions[@]} )); then
  region="${regions[$((choice-1))]}"
  url="${region_urls[$region]}"
  filename="$(basename "$url")"
  tmpfile="${filename}.part"

  echo "Downloading \"$region\"..."

  if wget -q --show-progress -O "$tmpfile" "$url"; then
      # Disable traps during atomic rename to avoid interrupt halfway
      trap - INT TERM EXIT
      mv -f "$tmpfile" "$filename"
      tmpfile=""  # Clear only after successful move
      echo "Download completed successfully: $filename"
      # Re-enable cleanup traps
      trap cleanup EXIT
      trap cleanup INT TERM
  else
      echo "Download failed."
      exit 1
  fi
else
  echo "Invalid selection."
fi
