#!/bin/bash
# Real-time packet processing for the robot's NIC, kept across reboots and
# link changes. The Panda's 1 kHz commands come late (control_command_success_rate
# drops, then communication_constraints_violation) when the NIC's receive
# processing runs at normal priority (a Realtek r8169 has no IRQ thread of its
# own: its packets are handled by ksoftirqd / backlog_napi, SCHED_OTHER). This
# puts it in a threaded NAPI of its own at SCHED_FIFO 95 (below libfranka's
# communication thread, 99):
#
#   /usr/local/sbin/franka-nic-rt                     does it (now and then)
#   /etc/systemd/system/franka-nic-rt.service         at boot
#   /etc/NetworkManager/dispatcher.d/50-franka-nic-rt when the link comes up again
#                                                     (cable replugged, robot rebooted)
#
#   sudo host/setup-nic-rt.sh [IFACE]        install (IFACE: the robot's, default enp92s0)
#   sudo host/setup-nic-rt.sh --uninstall    remove it (the running setting stays until a reboot)
#        host/setup-nic-rt.sh --check [IFACE]
set -e

IFACE=enp92s0
MODE=install
for a in "$@"; do
  case "$a" in
    --uninstall) MODE=uninstall ;;
    --check) MODE=check ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) IFACE="$a" ;;
  esac
done

SCRIPT=/usr/local/sbin/franka-nic-rt
UNIT=/etc/systemd/system/franka-nic-rt.service
HOOK=/etc/NetworkManager/dispatcher.d/50-franka-nic-rt

check() {
  echo "== $IFACE"
  if [ ! -e "/sys/class/net/$IFACE" ]; then
    echo "no such interface (ip -br link: the robot's is the one with 172.16.0.x)"; return 1
  fi
  echo "threaded NAPI: $(cat /sys/class/net/$IFACE/threaded 2>/dev/null || echo '?')"
  local found=0
  while read -r cls prio comm; do
    found=1
    echo "$comm: $cls $prio"
    [ "$cls" = FF ] && [ "$prio" -ge 90 ] || { echo "  -> not real time: sudo $SCRIPT $IFACE"; return 1; }
  done < <(ps -eLo cls,rtprio,comm | awk -v p="napi/$IFACE" 'index($3, p) == 1')
  [ $found = 1 ] || { echo "no napi/$IFACE thread: sudo $SCRIPT $IFACE"; return 1; }
  echo "ok"
}

if [ $MODE = check ]; then
  check
  exit $?
fi

if [ "$(id -u)" != 0 ]; then
  echo "run it with sudo: sudo $0 $*"; exit 1
fi

if [ $MODE = uninstall ]; then
  systemctl disable --now franka-nic-rt.service 2>/dev/null || true
  rm -f "$UNIT" "$HOOK" "$SCRIPT"
  systemctl daemon-reload
  echo "removed (the running setting stays until a reboot)"
  exit 0
fi

if [ ! -e "/sys/class/net/$IFACE" ]; then
  echo "no interface $IFACE; the robot's (172.16.0.x):"; ip -br addr | grep -v '^lo'
  echo "then: sudo $0 <IFACE>"; exit 1
fi

cat > "$SCRIPT" <<'EOF'
#!/bin/sh
# Real-time packet processing for the robot's NIC (robot-station host/setup-nic-rt.sh):
# a threaded NAPI, its thread at SCHED_FIFO 95.
IF=${1:-enp92s0}
[ -e /sys/class/net/$IF/threaded ] || exit 0
echo 1 > /sys/class/net/$IF/threaded
T=
for i in 1 2 3 4 5 6 7 8 9 10; do
  T=$(pgrep "^napi/$IF")
  [ -n "$T" ] && break
  sleep 0.5
done
for t in $T; do chrt -f -p 95 "$t"; done
logger -t franka-nic-rt "$IF: threaded NAPI at SCHED_FIFO 95 ($T)"
EOF
chmod 755 "$SCRIPT"

cat > "$UNIT" <<EOF
[Unit]
Description=Real-time packet processing for the robot's NIC ($IFACE)
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=$SCRIPT $IFACE

[Install]
WantedBy=multi-user.target
EOF

if [ -d /etc/NetworkManager/dispatcher.d ]; then
  cat > "$HOOK" <<EOF
#!/bin/sh
# robot-station host/setup-nic-rt.sh: again whenever the robot's link comes up
[ "\$1" = "$IFACE" ] && [ "\$2" = up ] && $SCRIPT $IFACE
exit 0
EOF
  chmod 755 "$HOOK"
else
  echo "(no NetworkManager: after replugging the robot's cable, run sudo $SCRIPT $IFACE)"
fi

systemctl daemon-reload
systemctl enable --now franka-nic-rt.service
echo "installed: $SCRIPT, $UNIT${HOOK:+, $HOOK}"
check
