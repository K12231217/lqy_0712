# IMU660RB Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Switch application-level gyroscope use from IMU660RA to the existing IMU660RB driver without changing yaw or race behavior.

**Architecture:** Keep the existing `imu660.c` calibration and integration boundary intact. Replace only the device API calls and global sample symbol; both device headers and the RB driver source are already included by the project.

**Tech Stack:** STC32G144K C, Seekfree IMU660RB driver, Keil C251/UV4, PowerShell static checks

---

### Task 1: Switch application calls to IMU660RB

**Files:**
- Modify: `project/code/Init.c:26`
- Modify: `project/code/imu660.c:46`
- Modify: `project/user/main.c:38`

- [ ] **Step 1: Run the static migration check and verify it fails**

Run:

```powershell
$hits = rg -n "imu660ra" project/code/Init.c project/code/imu660.c project/user/main.c
if ($LASTEXITCODE -eq 0) { $hits; exit 1 }
```

Expected: exit 1 with the existing IMU660RA initialization, sampling, sample-variable, conversion, or debug-comment references.

- [ ] **Step 2: Replace the device API symbols**

Apply these exact mappings in the three application files:

```c
imu660ra_init                -> imu660rb_init
imu660ra_get_gyro            -> imu660rb_get_gyro
imu660ra_gyro_z              -> imu660rb_gyro_z
imu660ra_gyro_transition     -> imu660rb_gyro_transition
```

Do not change calibration sample count, delays, deadband, integration interval, yaw scale, or call ordering.

- [ ] **Step 3: Run the static migration check and verify it passes**

Run:

```powershell
$hits = rg -n "imu660ra" project/code/Init.c project/code/imu660.c project/user/main.c
if ($LASTEXITCODE -eq 0) { $hits; exit 1 }
rg -n "imu660rb_(init|get_gyro|gyro_z|gyro_transition)" project/code/Init.c project/code/imu660.c project/user/main.c
```

Expected: no RA references; RB initialization, sampling, Z-axis sample, and conversion references are present.

- [ ] **Step 4: Compile the affected units**

Run C251 from `project/mdk` for `../code/Init.c`, `../code/imu660.c`, and `../user/main.c` using the project include directories.

Expected: each compilation completes with 0 errors.

- [ ] **Step 5: Rebuild and inspect the working-tree diff**

Run the Keil target rebuild, then:

```powershell
git diff --check
git diff -- project/code/Init.c project/code/imu660.c project/user/main.c
```

Expected: Keil reports 0 errors, and the diff contains only RA-to-RB symbol changes.

- [ ] **Step 6: Commit the focused migration**

```powershell
git add -- project/code/Init.c project/code/imu660.c project/user/main.c
git commit -m "feat: 切换至 IMU660RB 陀螺仪"
```

Do not stage the user-owned `project/code/sevro.c` or `project/code/sevro.h` status entries.

- [ ] **Step 7: Push and verify the remote head**

```powershell
git push origin main
git ls-remote origin refs/heads/main
```

Expected: `origin/main` points to the local migration commit. Report physical initialization and yaw-sign checks as HIL-blocked until tested on the car.
