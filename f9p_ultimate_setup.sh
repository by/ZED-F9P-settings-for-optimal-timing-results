#!/usr/bin/env bash
# f9p_ultimate_setup.sh — Configure ZED-F9P as a UTC-aligned PPS time source via gpsd + NTPsec
#
# HW: Raspberry Pi 5 + ZED-F9P (HPG 1.51 / PROTVER 27.50) + u-blox ANN-MB-00-00 (5 m RG-174)
# GOAL: TMODE3 Fixed LLH; PPS 1 Hz, UTC time grid, rising edge; gpsd provides SHM(0) time,
#       NTPsec disciplines kernel PPS. Minimal UBX on UART1; all other ports silenced.
#
# Notes:
# - We use VALSET layer bitmask: 1=RAM, 2=BBR, 4=FLASH. We write to 7 (RAM+BBR+FLASH): apply NOW + persist.
# - Message enables use VAL keys (CFG-MSGOUT-UBX_…_UART1). No legacy CFG-MSG needed.
# - Baud flip breaks the live link → do it LAST, then restart gpsd to re-probe.
# - Fixed position accuracy affects reported EPX/EPY/EPV only, not PPS timing.

set -euo pipefail

# ---- ubxtool via gpsd ----
UBX="ubxtool -P 27.50 -w 0.5 -v 0"
TARGET_BAUD=460800                       # 115200 is fine; 460800 gives headroom

# ---- User constants ----
LAT_D="49"                               # adjust to your location
LON_D="8"                                # adjust to your location
HEIGHT_M="1"                             # HAE, not MSL; adjust to your location

ANT_CABLE_DELAY_NS=25                    # ~25 ns for 5 m RG-174 (~0.66 VF)
TP_PERIOD_US=1000000                     # 1 Hz
TP_LEN_US=50000                          # 50 ms high pulse
TIME_REF_UTC=0                           # CFG-RATE-TIMEREF: 0 = UTC

DYNMODEL_STATIONARY=2
MIN_ELEV_DEG=20
MIN_CNO_DBHZ=15

# Fixed position accuracy (0.1 mm units): 1000=0.1 m, 5000=0.5 m, 10000=1.0 m
FIXEDPOS_ACC=10000

# Extra PPS user delay (ns). Keep 0 unless measured vs a reference.
USER_DELAY_NS=0

# ---- Derive u-blox fixed-point ints for LLH/height ----
export LAT_D LON_D HEIGHT_M
eval "$(
python3 - <<'PY'
from decimal import Decimal, ROUND_HALF_EVEN, getcontext
import os
getcontext().prec = 50
lat = Decimal(os.environ['LAT_D'])
lon = Decimal(os.environ['LON_D'])
h_m = Decimal(os.environ['HEIGHT_M'])

def split_deg(dd):
    nano = (dd * Decimal('1e9')).to_integral_value(rounding=ROUND_HALF_EVEN)
    hi = int(nano // 100) if nano >= 0 else -int((-nano)//100)
    hp = int(nano - hi*100)
    if hp >= 100: hi += 1; hp -= 100
    if hp <= -100: hi -= 1; hp += 100
    return hi, hp

def split_height(hm):
    total_01mm = (hm * Decimal('10000')).to_integral_value(rounding=ROUND_HALF_EVEN)
    cm = int(total_01mm // 100) if total_01mm >= 0 else -int((-total_01mm)//100)
    hp = int(total_01mm - cm*100)
    if hp >= 100: cm += 1; hp -= 100
    if hp <= -100: cm -= 1; hp += 100
    return cm, hp

lat_e7, lat_hp = split_deg(lat)
lon_e7, lon_hp = split_deg(lon)
h_cm,  h_hp  = split_height(h_m)

print(f"LAT={lat_e7}")
print(f"LON={lon_e7}")
print(f"HEIGHT={h_cm}")
print(f"LAT_HP={lat_hp}")
print(f"LON_HP={lon_hp}")
print(f"HEIGHT_HP={h_hp}")
PY
)"

# ---- Logging ----
LOGFILE="$HOME/f9p_setup.log"
exec > >(tee -a "$LOGFILE") 2>&1

ok()   { printf " OK\n"; }
nak()  { printf " NAK\n"; }
ign()  { printf " IGN\n"; }
err()  { printf " ERR: %s\n" "$1"; }
hdr()  { printf "\n== %s ==\n" "$1"; }

ubx_try() {  # IGN unknown/not found (firmware variance)
  local what="$1"; shift
  printf " - %-45s" "$what"
  local out rc=0
  out=$("$@" 2>&1) || rc=$?
  if ((rc)); then
    if grep -qiE "item .*unknown|not found" <<<"$out"; then ign; return 0; fi
    err "$out"; return 1
  fi
  if grep -q "NAK" <<<"$out"; then nak; else ok; fi
}

# Write VAL keys to RAM+BBR+FLASH (7). Fallback to 6, then 1 if needed.
ubx_set_all() {
  local item="$1" val="$2"
  printf " - %-45s" "${item}=${val} (RAM+BBR+FLASH)"
  local out rc=0
  out="$($UBX -z "${item},${val},7" 2>&1)" || rc=$?
  if ((rc)) || grep -q "NAK" <<<"$out"; then
    out="$($UBX -z "${item},${val},6" 2>&1)" || true
    if grep -q "NAK" <<<"$out"; then
      out="$($UBX -z "${item},${val},1" 2>&1)" || rc=1
    fi
  fi
  if ((rc)); then err "$out"; return 1; fi
  if grep -q "NAK" <<<"$out"; then nak; else ok; fi
}

# ---------------- START -----------------------------
hdr "Connecting (via gpsd)"
if ! $UBX -p MON-VER >/dev/null 2>&1; then
  echo "ERROR: ubxtool cannot reach receiver via gpsd. Is gpsd running?"; exit 1
fi
echo "Receiver reachable."

# Constellations (lean start; enable GLONASS/BeiDou later if you want steadier DOP)
hdr "Constellations"
ubx_set_all CFG-SIGNAL-GPS_ENA              1
ubx_set_all CFG-SIGNAL-GAL_ENA              1
ubx_set_all CFG-SIGNAL-QZSS_ENA             1
ubx_set_all CFG-SIGNAL-GLO_ENA              0
ubx_set_all CFG-SIGNAL-BDS_ENA              0
ubx_set_all CFG-SIGNAL-SBAS_ENA             0

# Engine rates & UTC base
hdr "Rates"
ubx_set_all CFG-RATE-MEAS                   1000
ubx_set_all CFG-RATE-NAV                    1
ubx_set_all CFG-RATE-TIMEREF                $TIME_REF_UTC # 0 = UTC

# Nav model & input masks
hdr "Nav Model & Filters"
ubx_set_all CFG-NAVSPG-DYNMODEL             $DYNMODEL_STATIONARY
ubx_set_all CFG-NAVSPG-UTCSTANDARD          0 # 0 = Auto (ensures TPV.time/SHM(0) comes alive)
ubx_set_all CFG-NAVSPG-INFIL_MINELEV        $MIN_ELEV_DEG
ubx_set_all CFG-NAVSPG-INFIL_MINCNO         $MIN_CNO_DBHZ

# Antenna supervisor & interference monitor (safe where supported)
hdr "Antenna & Interference"
ubx_try "ITFM (anti-jam) enable"            $UBX -z CFG-ITFM-ENABLE,1,7
ubx_try "Active antenna: volt ctrl"         $UBX -z CFG-HW-ANT_CFG_VOLTCTRL,1,7
ubx_try "Active antenna: short det"         $UBX -z CFG-HW-ANT_CFG_SHORTDET,1,7
ubx_try "Active antenna: open det"          $UBX -z CFG-HW-ANT_CFG_OPENDET,1,7
ubx_try "Active antenna: power-down"        $UBX -z CFG-HW-ANT_CFG_PWRDOWN,1,7
ubx_try "Active antenna: auto-recover"      $UBX -z CFG-HW-ANT_CFG_RECOVER,1,7

# Ports: allow only UBX on UART1; shut USB/I2C/SPI completely (IN+OUT)
hdr "Ports & Protocols"
# UART1 input protocols
ubx_set_all CFG-UART1INPROT-UBX              1
ubx_set_all CFG-UART1INPROT-NMEA             0
ubx_set_all CFG-UART1INPROT-RTCM3X           0
# UART1 output protocols
ubx_set_all CFG-UART1OUTPROT-UBX             1
ubx_set_all CFG-UART1OUTPROT-NMEA            0
ubx_set_all CFG-UART1OUTPROT-RTCM3X          0

# USB completely off (both directions)
ubx_set_all CFG-USBINPROT-UBX                0
ubx_set_all CFG-USBINPROT-NMEA               0
ubx_set_all CFG-USBINPROT-RTCM3X             0
ubx_set_all CFG-USBOUTPROT-UBX               0
ubx_set_all CFG-USBOUTPROT-NMEA              0
ubx_set_all CFG-USBOUTPROT-RTCM3X            0

# I2C completely off (both directions)
ubx_set_all CFG-I2CINPROT-UBX                0
ubx_set_all CFG-I2CINPROT-NMEA               0
ubx_set_all CFG-I2CINPROT-RTCM3X             0
ubx_set_all CFG-I2COUTPROT-UBX               0
ubx_set_all CFG-I2COUTPROT-NMEA              0
ubx_set_all CFG-I2COUTPROT-RTCM3X            0

# SPI completely off (both directions)
ubx_set_all CFG-SPIINPROT-UBX                0
ubx_set_all CFG-SPIINPROT-NMEA               0
ubx_set_all CFG-SPIINPROT-RTCM3X             0
ubx_set_all CFG-SPIOUTPROT-UBX               0
ubx_set_all CFG-SPIOUTPROT-NMEA              0
ubx_set_all CFG-SPIOUTPROT-RTCM3X            0

# TMODE3 Fixed LLH — atomic VAL configuration (apply now + persist)
hdr "TMODE3 Fixed LLH"
ubx_set_all CFG-TMODE-MODE                   0 # disable first to update fields safely
ubx_set_all CFG-TMODE-POS_TYPE               1 # 1 = LLH
ubx_set_all CFG-TMODE-LAT                    "$LAT"
ubx_set_all CFG-TMODE-LON                    "$LON"
ubx_set_all CFG-TMODE-HEIGHT                 "$HEIGHT"
ubx_set_all CFG-TMODE-LAT_HP                 "$LAT_HP"
ubx_set_all CFG-TMODE-LON_HP                 "$LON_HP"
ubx_set_all CFG-TMODE-HEIGHT_HP              "$HEIGHT_HP"
ubx_set_all CFG-TMODE-FIXED_POS_ACC          "$FIXEDPOS_ACC"
ubx_set_all CFG-TMODE-MODE                   2 # 2 = Fixed

# Messages (UART1 only, VAL keys). With MEAS=1000 and NAV=1, rate 1 => 1 Hz.
hdr "Messages (UART1 only, VAL keys)"
# Ensure TPV time is valid for SHM(0)
ubx_set_all CFG-MSGOUT-UBX_NAV_PVT_UART1      1
ubx_set_all CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1  1
# Optional but harmless:
#ubx_set_all CFG-MSGOUT-UBX_NAV_TIMEGPS_UART1  1
# TDOP + satellite list (for ntpviz)
ubx_set_all CFG-MSGOUT-UBX_NAV_DOP_UART1      1
ubx_set_all CFG-MSGOUT-UBX_NAV_SAT_UART1      1
# Survey-in / “Fixed (surveyed)” banner
ubx_set_all CFG-MSGOUT-UBX_NAV_SVIN_UART1     1
# (Ports off already, but explicitly zero on others too if you like:)
ubx_set_all CFG-MSGOUT-UBX_NAV_PVT_USB        0
ubx_set_all CFG-MSGOUT-UBX_NAV_TIMEUTC_USB    0
ubx_set_all CFG-MSGOUT-UBX_NAV_DOP_USB        0
ubx_set_all CFG-MSGOUT-UBX_NAV_SAT_USB        0
ubx_set_all CFG-MSGOUT-UBX_NAV_SVIN_USB       0
ubx_set_all CFG-MSGOUT-UBX_NAV_PVT_I2C        0
ubx_set_all CFG-MSGOUT-UBX_NAV_TIMEUTC_I2C    0
ubx_set_all CFG-MSGOUT-UBX_NAV_DOP_I2C        0
ubx_set_all CFG-MSGOUT-UBX_NAV_SAT_I2C        0
ubx_set_all CFG-MSGOUT-UBX_NAV_SVIN_I2C       0
ubx_set_all CFG-MSGOUT-UBX_NAV_PVT_SPI        0
ubx_set_all CFG-MSGOUT-UBX_NAV_TIMEUTC_SPI    0
ubx_set_all CFG-MSGOUT-UBX_NAV_DOP_SPI        0
ubx_set_all CFG-MSGOUT-UBX_NAV_SAT_SPI        0
ubx_set_all CFG-MSGOUT-UBX_NAV_SVIN_SPI       0

# PPS (TP1): UTC grid, TOW-aligned, rising, cable delay
hdr "PPS (TP1)"
ubx_set_all CFG-TP-TP1_ENA                    1
ubx_set_all CFG-TP-PULSE_DEF                  0 # Period
ubx_set_all CFG-TP-PULSE_LENGTH_DEF           1 # Length
ubx_set_all CFG-TP-PERIOD_TP1                 $TP_PERIOD_US
ubx_set_all CFG-TP-LEN_TP1                    $TP_LEN_US
ubx_set_all CFG-TP-ALIGN_TO_TOW_TP1           1
ubx_set_all CFG-TP-USE_LOCKED_TP1             1
ubx_set_all CFG-TP-POL_TP1                    1
ubx_set_all CFG-TP-ANT_CABLEDELAY             $ANT_CABLE_DELAY_NS
ubx_set_all CFG-TP-TIMEGRID_TP1               0 # 0 = UTC grid
if [[ "${USER_DELAY_NS}" != "0" ]]; then
  ubx_set_all CFG-TP-USER_DELAY_TP1 "${USER_DELAY_NS}" || true
fi
ubx_try "TP1 Smoothing (opt)" $UBX -z CFG-TP-SMOOTHING_TP1,1,7

# Persist (modern + legacy)
hdr "Persist"
ubx_try "SAVE (modern)" $UBX -p SAVE
ubx_try "CFG-CFG Save (legacy)" $UBX -c 06,09,00,FF,FF,00,00

# Finalize: change UART1 baud LAST, then restart gpsd to re-autobaud
hdr "Finalize: set UART1 baud then restart gpsd"
ubx_try "UART1 baud=${TARGET_BAUD} (RAM+BBR+FLASH)" \
  $UBX -z CFG-UART1-BAUDRATE,$TARGET_BAUD,7 || true

# Quick verify (through gpsd)
hdr "Quick Verify"
printf " - %-45s" "TMODE3 summary"; $UBX -p CFG-TMODE3 2>/dev/null | \
  awk '/UBX-CFG-TMODE3:/{p=1;next} p&&/flags|mode|ecef|lla|fixedPosAcc/{print}'; echo
printf " - %-45s" "TP1 Cable Delay"; $UBX -g CFG-TP-ANT_CABLEDELAY >/dev/null 2>&1 && ok || nak
printf " - %-45s" "NAV-SVIN poll";   $UBX -p NAV-SVIN >/dev/null 2>&1 && ok || nak

echo -e "\nSummary: DONE"
echo "Hints: cgps -s  ·  sudo ppstest /dev/pps0  ·  ntpshmmon -o 5  ·  gpspipe -w -n 10 | grep TPV"
