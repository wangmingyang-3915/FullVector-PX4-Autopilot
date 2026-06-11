# Fullvector RC Switch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make fullvector the default controller while allowing an RC AUX switch to hand actuator control back to the native PX4 controller and center the tilt servos.

**Architecture:** `fullvector_control` owns the RC switch decision and publishes a small uORB status topic. `control_allocator` uses that status topic, with freshness timeout, to decide whether to yield actuator publication to fullvector or resume native PX4 actuator output.

**Tech Stack:** PX4 C++, uORB messages, module parameters, PowerShell static regression check.

---

### Task 1: Static Regression Check

**Files:**
- Create: `test/fullvector_control/check_rc_switch_static.ps1`

- [x] **Step 1: Write a failing static test**

The script checks that `FullvectorControlStatus.msg` exists, is registered in `msg/CMakeLists.txt`, is published by `fullvector_control`, and is consumed by `control_allocator`.

- [x] **Step 2: Run the test and confirm RED**

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File test/fullvector_control/check_rc_switch_static.ps1`

Expected before implementation: FAIL because the new status topic and RC switch code are absent.

### Task 2: Fullvector Status Topic

**Files:**
- Create: `msg/FullvectorControlStatus.msg`
- Modify: `msg/CMakeLists.txt`

- [x] **Step 1: Add uORB message fields**

Fields: `timestamp`, `fullvector_active`, `native_requested`, `rc_switch_valid`, `rc_switch_value`.

- [x] **Step 2: Register the message**

Add `FullvectorControlStatus.msg` to `msg_files`.

### Task 3: Fullvector RC Switch Logic

**Files:**
- Modify: `src/modules/fullvector_control/fullvector_control.hpp`
- Modify: `src/modules/fullvector_control/fullvector_control.cpp`
- Modify: `src/modules/fullvector_control/module.yaml`

- [x] **Step 1: Add parameters**

Add `FV_RC_SW_EN`, `FV_RC_SW_CH`, `FV_RC_SW_THR`, `FV_RC_SW_REV`.

- [x] **Step 2: Publish status**

Publish current fullvector ownership each control cycle.

- [x] **Step 3: Center servos during native handoff**

When native is requested, reset PID state, stop motor publication, and publish neutral tilt servos briefly so the handoff is mechanical-safe.

### Task 4: Control Allocator Arbitration

**Files:**
- Modify: `src/modules/control_allocator/ControlAllocator.hpp`
- Modify: `src/modules/control_allocator/ControlAllocator.cpp`

- [x] **Step 1: Subscribe to fullvector status**

Add `fullvector_control_status` subscription and cache.

- [x] **Step 2: Replace FV_ENABLE-only arbitration**

Only return early from `publish_actuator_controls()` when `fullvector_active` is true and the status timestamp is fresh.

### Task 5: Verification

**Files:**
- Test: `test/fullvector_control/check_rc_switch_static.ps1`

- [x] **Step 1: Re-run static regression check**

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File test/fullvector_control/check_rc_switch_static.ps1`

Expected: PASS.

- [ ] **Step 2: Run a lightweight build/listener check if available**

Prefer generating uORB headers or running a configured PX4 build target if the local environment supports it.
