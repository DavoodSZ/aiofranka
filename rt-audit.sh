#!/usr/bin/env bash
# rt_audit.sh — System-level real-time audit for 1kHz torque control
# Run as root: sudo bash rt_audit.sh
# Target: improbable014, aiofranka on Franka Panda (172.16.0.2)

set -euo pipefail

RED='\033[0;31m'
YEL='\033[0;33m'
GRN='\033[0;32m'
CYN='\033[0;36m'
BLD='\033[1m'
RST='\033[0m'

FRANKA_IP="172.16.0.2"
RT_CPU=31  # cpu pin from your benchmark

pass()  { echo -e "  ${GRN}✓${RST} $1"; }
warn()  { echo -e "  ${YEL}⚠${RST} $1"; }
fail()  { echo -e "  ${RED}✗${RST} $1"; }
info()  { echo -e "  ${CYN}ℹ${RST} $1"; }
header(){ echo -e "\n${BLD}[$1]${RST}"; }

# ─────────────────────────────────────────────
header "1. Kernel & PREEMPT_RT"
# ─────────────────────────────────────────────
KVER=$(uname -r)
echo "  Kernel: $KVER"

if grep -qi "preempt_rt\|preempt rt" /boot/config-"$KVER" 2>/dev/null; then
    pass "PREEMPT_RT detected in kernel config"
elif uname -v | grep -qi "preempt"; then
    warn "Kernel has PREEMPT but not full PREEMPT_RT — tail latency will suffer"
else
    fail "No PREEMPT_RT — this is the single biggest improvement you can make"
    info "Consider: apt install linux-image-rt-amd64 (Debian) or linux-lowlatency (Ubuntu)"
fi

# Check preempt model
if [ -f /sys/kernel/realtime ]; then
    RT_VAL=$(cat /sys/kernel/realtime)
    [[ "$RT_VAL" == "1" ]] && pass "Kernel reports realtime=1" || warn "realtime=$RT_VAL"
fi

# ─────────────────────────────────────────────
header "2. CPU Isolation & Scheduling (core $RT_CPU)"
# ─────────────────────────────────────────────
CMDLINE=$(cat /proc/cmdline)
echo "  Kernel cmdline: $CMDLINE"

# isolcpus
if echo "$CMDLINE" | grep -qP "isolcpus=.*\b${RT_CPU}\b"; then
    pass "Core $RT_CPU is in isolcpus"
else
    fail "Core $RT_CPU is NOT in isolcpus — kernel may schedule other tasks here"
    info "Add to GRUB: isolcpus=$RT_CPU"
fi

# nohz_full
if echo "$CMDLINE" | grep -qP "nohz_full=.*\b${RT_CPU}\b"; then
    pass "Core $RT_CPU is in nohz_full (tickless)"
else
    warn "Core $RT_CPU is NOT in nohz_full — timer ticks will cause jitter"
    info "Add to GRUB: nohz_full=$RT_CPU"
fi

# rcu_nocbs
if echo "$CMDLINE" | grep -qP "rcu_nocbs=.*\b${RT_CPU}\b"; then
    pass "Core $RT_CPU is in rcu_nocbs"
else
    warn "Core $RT_CPU is NOT in rcu_nocbs — RCU callbacks may interrupt RT thread"
    info "Add to GRUB: rcu_nocbs=$RT_CPU"
fi

# Check what's currently running on the RT core
echo ""
info "Processes currently on core $RT_CPU:"
ps -eo pid,comm,psr --no-headers | awk -v cpu="$RT_CPU" '$3 == cpu {printf "      PID %-6s  %s\n", $1, $2}'
COUNT=$(ps -eo psr --no-headers | awk -v cpu="$RT_CPU" '$1 == cpu' | wc -l)
if [ "$COUNT" -le 2 ]; then
    pass "Only $COUNT process(es) on core $RT_CPU"
else
    warn "$COUNT processes on core $RT_CPU — consider migrating them"
fi

# ─────────────────────────────────────────────
header "3. CPU Frequency Scaling"
# ─────────────────────────────────────────────
GOV_PATH="/sys/devices/system/cpu/cpu${RT_CPU}/cpufreq/scaling_governor"
if [ -f "$GOV_PATH" ]; then
    GOV=$(cat "$GOV_PATH")
    if [ "$GOV" == "performance" ]; then
        pass "Governor on core $RT_CPU: performance"
    else
        fail "Governor on core $RT_CPU: $GOV — should be 'performance'"
        info "Fix: echo performance > $GOV_PATH"
    fi
    CURFREQ=$(cat /sys/devices/system/cpu/cpu${RT_CPU}/cpufreq/scaling_cur_freq 2>/dev/null || echo "?")
    MAXFREQ=$(cat /sys/devices/system/cpu/cpu${RT_CPU}/cpufreq/scaling_max_freq 2>/dev/null || echo "?")
    info "Current freq: ${CURFREQ}kHz / Max: ${MAXFREQ}kHz"
else
    warn "No cpufreq sysfs for core $RT_CPU (may be managed by firmware)"
fi

# Check intel_pstate / boost
if [ -f /sys/devices/system/cpu/intel_pstate/no_turbo ]; then
    NT=$(cat /sys/devices/system/cpu/intel_pstate/no_turbo)
    if [ "$NT" == "1" ]; then
        info "Turbo boost disabled (good for latency consistency)"
    else
        warn "Turbo boost enabled — can cause frequency transition jitter"
        info "Disable: echo 1 > /sys/devices/system/cpu/intel_pstate/no_turbo"
    fi
fi

# C-states
if command -v cpupower &>/dev/null; then
    info "Idle driver: $(cpupower idle-info 2>/dev/null | grep 'driver:' | awk '{print $NF}')"
fi
if echo "$CMDLINE" | grep -q "idle=poll\|processor.max_cstate=0\|intel_idle.max_cstate=0"; then
    pass "Deep C-states appear disabled via cmdline"
else
    warn "Deep C-states may be active — wakeup latency can cause spikes"
    info "Add to GRUB: processor.max_cstate=1 intel_idle.max_cstate=0"
    info "Or at runtime: echo 0 > /dev/cpu_dma_latency (hold fd open)"
fi

# ─────────────────────────────────────────────
header "4. Network Interface (Franka link to $FRANKA_IP)"
# ─────────────────────────────────────────────

# Find the NIC for the Franka subnet
FRANKA_NIC=$(ip route get "$FRANKA_IP" 2>/dev/null | grep -oP 'dev \K\S+' || echo "")
if [ -z "$FRANKA_NIC" ]; then
    fail "Cannot determine NIC for $FRANKA_IP"
else
    info "Franka NIC: $FRANKA_NIC"

    # Link speed
    SPEED=$(ethtool "$FRANKA_NIC" 2>/dev/null | grep "Speed:" | awk '{print $2}')
    if [ -n "$SPEED" ]; then
        info "Link speed: $SPEED"
        if echo "$SPEED" | grep -q "1000"; then
            pass "Gigabit link"
        else
            warn "Not gigabit — Franka requires 1Gbps direct connection"
        fi
    fi

    # Interrupt coalescing — THE BIG ONE
    echo ""
    info "Interrupt coalescing settings:"
    COAL=$(ethtool -c "$FRANKA_NIC" 2>/dev/null)
    if [ -n "$COAL" ]; then
        echo "$COAL" | grep -E "rx-usecs|rx-frames|adaptive" | while read -r line; do
            echo "      $line"
        done

        RX_USECS=$(echo "$COAL" | grep "^rx-usecs:" | awk '{print $2}')
        ADAPTIVE=$(echo "$COAL" | grep "Adaptive RX:" | awk '{print $3}')

        if [ "$ADAPTIVE" == "on" ]; then
            fail "Adaptive RX coalescing is ON — this causes variable latency"
            info "Fix: ethtool -C $FRANKA_NIC adaptive-rx off"
        fi

        if [ -n "$RX_USECS" ] && [ "$RX_USECS" != "0" ]; then
            fail "rx-usecs=$RX_USECS — NIC batches interrupts, adding up to ${RX_USECS}us latency"
            info "Fix: ethtool -C $FRANKA_NIC rx-usecs 0 rx-frames 1"
        elif [ "$RX_USECS" == "0" ]; then
            pass "rx-usecs=0 (immediate interrupt on packet arrival)"
        fi
    else
        warn "Cannot read coalescing settings (ethtool -c failed)"
    fi

    # Ring buffer size
    echo ""
    RING=$(ethtool -g "$FRANKA_NIC" 2>/dev/null)
    if [ -n "$RING" ]; then
        info "RX ring buffer:"
        echo "$RING" | grep -A1 "Current" | grep "RX:" | while read -r line; do
            echo "      $line"
        done
    fi

    # IRQ affinity
    echo ""
    info "IRQ affinity for $FRANKA_NIC:"
    IRQS=$(grep "$FRANKA_NIC" /proc/interrupts 2>/dev/null | awk '{print $1}' | tr -d ':')
    if [ -z "$IRQS" ]; then
        # Try without NIC name — some drivers use queue names
        IRQS=$(grep -i "$FRANKA_NIC" /proc/interrupts 2>/dev/null | awk '{print $1}' | tr -d ':')
    fi

    if [ -n "$IRQS" ]; then
        for IRQ in $IRQS; do
            AFF=$(cat /proc/irq/$IRQ/smp_affinity_list 2>/dev/null || echo "?")
            info "  IRQ $IRQ → CPU(s): $AFF"
            if echo "$AFF" | grep -qP "^\s*${RT_CPU}\s*$"; then
                pass "  IRQ $IRQ pinned to RT core $RT_CPU"
            else
                warn "  IRQ $IRQ not on core $RT_CPU"
                info "  Fix: echo $RT_CPU > /proc/irq/$IRQ/smp_affinity_list"
            fi
        done
    else
        warn "Cannot find IRQs for $FRANKA_NIC in /proc/interrupts"
        info "Check: cat /proc/interrupts | grep -i eth  (or your NIC driver name)"
    fi

    # Check if irqbalance is running (it will undo manual affinity)
    if pgrep -x irqbalance &>/dev/null; then
        fail "irqbalance is running — it will override IRQ affinity settings"
        info "Fix: systemctl stop irqbalance && systemctl disable irqbalance"
    else
        pass "irqbalance is not running"
    fi
fi

# ─────────────────────────────────────────────
header "5. Memory & Kernel Tweaks"
# ─────────────────────────────────────────────

# THP
THP=$(cat /sys/kernel/mm/transparent_hugepage/enabled 2>/dev/null || echo "?")
if echo "$THP" | grep -q "\[never\]"; then
    pass "Transparent Huge Pages: disabled"
else
    warn "Transparent Huge Pages: $THP — compaction can cause latency spikes"
    info "Fix: echo never > /sys/kernel/mm/transparent_hugepage/enabled"
fi

# Swappiness
SWAP=$(cat /proc/sys/vm/swappiness)
if [ "$SWAP" -le 10 ]; then
    pass "vm.swappiness=$SWAP"
else
    warn "vm.swappiness=$SWAP — consider lowering to 1"
fi

# RT throttling
RT_PERIOD=$(cat /proc/sys/kernel/sched_rt_runtime_us 2>/dev/null || echo "?")
RT_RUNTIME=$(cat /proc/sys/kernel/sched_rt_period_us 2>/dev/null || echo "?")
info "RT throttling: runtime=${RT_PERIOD}us / period=${RT_RUNTIME}us"
if [ "$RT_PERIOD" == "-1" ]; then
    pass "RT throttling disabled (unlimited RT CPU time)"
else
    warn "RT throttling is active — can cause periodic stalls"
    info "Fix: echo -1 > /proc/sys/kernel/sched_rt_runtime_us"
fi

# Kernel watchdog
NMI_WD=$(cat /proc/sys/kernel/nmi_watchdog 2>/dev/null || echo "?")
if [ "$NMI_WD" == "0" ]; then
    pass "NMI watchdog disabled"
else
    warn "NMI watchdog=$NMI_WD — causes periodic PMI interrupts"
    info "Fix: echo 0 > /proc/sys/kernel/nmi_watchdog"
fi

# ─────────────────────────────────────────────
header "6. Power Management"
# ─────────────────────────────────────────────

# SMI (System Management Interrupt) — the silent killer
if [ -f /sys/kernel/debug/x86/smi_count ]; then
    SMI=$(cat /sys/kernel/debug/x86/smi_count 2>/dev/null || echo "?")
    info "SMI count: $SMI (check again after benchmark — if it increases, SMIs are causing spikes)"
else
    info "Cannot read SMI count (mount debugfs or check BIOS for SMI disable option)"
fi

# ASPM
if echo "$CMDLINE" | grep -q "pcie_aspm=off"; then
    pass "PCIe ASPM disabled via cmdline"
else
    warn "PCIe ASPM may be active — can cause latency on NIC wakeup"
    info "Fix: add pcie_aspm=off to kernel cmdline"
fi

# ─────────────────────────────────────────────
header "7. Cyclictest Baseline (5s, quick check)"
# ─────────────────────────────────────────────
if command -v cyclictest &>/dev/null; then
    info "Running cyclictest on core $RT_CPU for 5 seconds..."
    CT_OUT=$(cyclictest -t1 -p80 -a"$RT_CPU" -i1000 -D5 -q 2>&1)
    echo "$CT_OUT" | tail -1
    MAX_LAT=$(echo "$CT_OUT" | grep "Max Latencies" | awk '{print $NF}')
    if [ -n "$MAX_LAT" ] && [ "$MAX_LAT" -lt 50 ]; then
        pass "Max wakeup latency: ${MAX_LAT}us (excellent)"
    elif [ -n "$MAX_LAT" ] && [ "$MAX_LAT" -lt 150 ]; then
        warn "Max wakeup latency: ${MAX_LAT}us (okay but room for improvement)"
    elif [ -n "$MAX_LAT" ]; then
        fail "Max wakeup latency: ${MAX_LAT}us (too high for reliable 1kHz)"
    fi
else
    warn "cyclictest not found — install rt-tests: apt install rt-tests"
fi

# ─────────────────────────────────────────────
header "8. Summary of Recommended GRUB Changes"
# ─────────────────────────────────────────────
echo ""
info "Suggested /etc/default/grub GRUB_CMDLINE_LINUX additions:"
echo -e "  ${CYN}isolcpus=$RT_CPU nohz_full=$RT_CPU rcu_nocbs=$RT_CPU${RST}"
echo -e "  ${CYN}processor.max_cstate=1 intel_idle.max_cstate=0${RST}"
echo -e "  ${CYN}pcie_aspm=off${RST}"
echo ""
info "Then: update-grub && reboot"
echo ""
info "Runtime fixes (apply now, non-persistent):"
echo -e "  ${CYN}echo performance > /sys/devices/system/cpu/cpu${RT_CPU}/cpufreq/scaling_governor${RST}"
echo -e "  ${CYN}echo never > /sys/kernel/mm/transparent_hugepage/enabled${RST}"
echo -e "  ${CYN}echo 0 > /proc/sys/kernel/nmi_watchdog${RST}"
echo -e "  ${CYN}echo -1 > /proc/sys/kernel/sched_rt_runtime_us${RST}"
if [ -n "$FRANKA_NIC" ]; then
    echo -e "  ${CYN}ethtool -C $FRANKA_NIC adaptive-rx off rx-usecs 0 rx-frames 1${RST}"
    echo -e "  ${CYN}systemctl stop irqbalance${RST}"
fi
echo ""
