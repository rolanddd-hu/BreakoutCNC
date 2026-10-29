# Hardware Validation Checklist (after the Step 95 safety rework)

The controller changes from Step 95 pass all simulator tests, but some behaviour depends on how the
RNR Eco Motion 2.0 board actually works. Those assumptions are marked **[ASSUMPTION]** below. Work
through the phases in order and stop at the first failure.

Record results in the "Result" column of the summary table at the end. If something fails, note the
exact `active_fault_message` text shown in the UI (status bar / fault banner): most new checks report
themselves through that message.

---

## Phase 0 - Preparation (machine safe)

- [ ] **Stepper drivers powered OFF, or motor cables unplugged.** Phases 1-6 need only the board: it
      generates step pulses and counts them whether or not motors are connected.
- [ ] Spindle / torch / laser power OFF (outputs are tested via the I/O LEDs only).
- [ ] A **hardwired** E-stop that cuts driver power exists and works. The software E-stop is a second
      layer, never the only one.
- [ ] Back up `breakout_mill_settings.ini` / `breakout_cut_settings.ini` (a `.bak` is now created on
      every save, but keep your own copy too).
- [ ] Release build is current: `cmake --build build --config Release`.
- [ ] Note: `script_engine_test.exe` no longer touches the board, but `rnr_hid_test.exe` **does**: it
      sends the reset/arm handshake. Only run it with drivers off and the app closed.

---

## Phase 1 - Board communication & watchdogs

**[ASSUMPTION] The board sends status reports continuously, at least every 500 ms.**
The new watchdog faults with *"MOTION BOARD STOPPED RESPONDING (no status reports)"* otherwise.

- [ ] With the app closed, run `build\Release\rnr_hid_test.exe`. Note "Total Reports Read ... (xx Hz)".
      Expect a steady rate (the protocol suggests ~200 Hz). **Must be well above 2 Hz.**
      Also note the "Reports: Input=.. Output=.. bytes" line (expected 49 / 27, or larger with a report ID).
- [ ] Start BreakoutMill (or BreakoutCut). Expect: connected, state **E-STOP**, no fault message.
- [ ] Wait 10 s in E-stop, press **RESET E-STOP**, wait 30 s idle.
      Expect: stays **Idle**, no "STOPPED RESPONDING" fault.
      - If it faults: the board does not report continuously while idle. Stop here and report the
        rate seen in `rnr_hid_test`.
- [ ] Unplug the USB cable while Idle. Expect: state **Disconnected**, message
      *"USB CONNECTION TO MOTION BOARD LOST"*. **RESET E-STOP** must be refused
      (*"Cannot reset: motion board is not connected"*).
- [ ] Plug back in, press **Connect USB**. Expect: E-STOP state, then reset works.

## Phase 2 - E-stop paths

- [ ] **ESC key** while Idle -> E-STOP. Also while typing in the MDI field (focus in a text box).
- [ ] UI **EMERGENCY STOP** button -> E-STOP.
- [ ] If an E-stop input pin is configured: press the physical E-stop -> E-STOP with
      *"E-STOP INPUT PIN TRIGGERED"*. With the button still pressed, **RESET E-STOP** must be refused
      (*"E-Stop input is still active"*). Release -> reset works.
- [ ] Same test with **Limit Override ON**: the E-stop input must still trigger (this was the bypass bug).
- [ ] **[ASSUMPTION] board fault flag 0x20 = hardware E-stop.** If the board has its own E-stop
      terminal, trigger it -> expect *"HARDWARE E-STOP REPORTED BY MOTION BOARD"* and reset refused
      while active. (If nothing happens, run `rnr_hid_test.exe` while holding it and note `Fault=0x..`.)
- [ ] Remote server (if enabled): `POST /api/estop` from the dashboard button -> E-STOP.
      Opening `http://<pc>:<port>/api/estop` in a browser (GET) must **not** E-stop (405).

## Phase 3 - Limit switches

For each configured limit input (press the switch by hand):

- [ ] Idle: press switch -> E-STOP with *"END-STOP HIT: X+ Limit (INn)"* (axis/side correct).
- [ ] While held: **RESET E-STOP** refused (*"... enable Limit Override to jog off"*).
- [ ] Turn on **Limit Override** -> reset works; jog works; release switch; turn override off.
- [ ] Restart the app with override left ON: it must come back **OFF** (no longer persisted).
- [ ] **[ASSUMPTION] board fault flag 0x10 = hardware limit.** Note whether pressing a limit also
      produces *"board hardware limit fault"* (fine either way, just record it).

## Phase 4 - Position feedback (drivers still off)

**[ASSUMPTION] `live_steps` in the status report = board step counters in the same units and sign as
`SetAxisPositions`.** The controller compares them with its own position whenever the board is idle.
It only trusts them after the first exact match. After that, any difference is "corrected" with the
message *"Position corrected from motion board feedback (x.xx mm)"*.

- [ ] After reset, jog X +10 mm (step mode), wait 1 s. Expect **no** "Position corrected" message.
- [ ] Repeat for Y, Z, A and both directions, and a diagonal MDI move `G91 G0 X5 Y5` / `G90`.
- [ ] Close the app, run `rnr_hid_test.exe`: the printed `Steps X=..` must equal
      DRO machine position x steps/mm from the last session (sign included).
      - If the sign is inverted or the scale differs, the correction logic would move the DRO
        wrongly: **stop and report** (the fix is in `processBoardReportLocked`).
- [ ] Change steps/mm for X in settings (Idle), apply, jog 10 mm, check no correction message, then
      restore the original value.

## Phase 5 - Streaming, flow control, feed hold, stop

**[ASSUMPTION] up to 3 slices (75 ms) may be sent ahead of the acknowledgement.**
Faults to watch for: *"MOTION BOARD STOPPED ACKNOWLEDGING MOTION DATA"* (acks lag > 1.5 s) and
*"USB WRITE FAILED WHILE STREAMING MOTION"*.

- [ ] Load a long air-cut program (several minutes, arcs + short segments, Z above stock), start it.
      Expect: runs to the end, no ack / write faults, progress highlight follows the program.
- [ ] During the run press **FEED HOLD**: DRO keeps moving briefly (deceleration) then stops.
      Press Cycle Start: resumes from where it stopped, no jump of the DRO.
- [ ] During the run press **STOP / REWIND**: decelerates, then Idle; no "Position corrected"
      message after it stops.
- [ ] Change the feed override slider during the run (50% -> 150% -> 100%): speed change within
      ~0.5 s, DRO never jumps.
- [ ] Program with `M0` in the middle: pauses with *"Program stop (M0/M1)"*, Cycle Start continues.
- [ ] **[ASSUMPTION] behaviour after `CancelSequence`.** Press **EMERGENCY STOP** mid-move, reset.
      Expect either no correction message, or one correction; DRO must then match `rnr_hid_test`
      steps. Note which happened. (If feedback was never validated in Phase 4, the axes are marked
      not homed with *"position uncertain, re-home the machine"* - that is intended.)

## Phase 6 - Homing & probing inputs (drivers still off, switches pressed by hand)

Tip: temporarily set `homing_max_travel_mm = 50` under `[Axis_0]` .. `[Axis_2]` in the settings
file so a failed search ends quickly (restore afterwards).

- [ ] **HOME** X: while "seeking", press the X home switch by hand -> pull-off move, axis shows
      homed, machine X = +pulloff (homing toward -).
- [ ] Home X again without pressing anything -> *"Homing X failed: home switch not found within 50 mm"*.
- [ ] Hold the home switch pressed, start homing -> it backs off first; release the switch ->
      continues with the seek.
- [ ] Axis without a home input configured -> *"Cannot home X: no home switch input is configured"*.
- [ ] **PROBE Z**: during the probe move touch the probe/plate input -> stops, retracts, Z offset set.
- [ ] Probe without touching -> *"Probing failed: no probe contact ..."*.
- [ ] Hold the probe input active, start probing -> refused (*"probe is already in contact"*).
- [ ] **[ASSUMPTION] `latched_steps` = position at probe trigger.** Compare the Z offset from two
      probes at different feeds; they should agree within ~0.02 mm at the same physical trigger.
- [ ] Program containing `G53 Z0` or `G28 G91 Z0` with Z not homed -> Cycle Start refused
      (*"... not homed. Home the machine first."*).

## Phase 7 - Outputs, spindle, VFD

- [ ] In E-STOP, try spindle / air assist / red pointer buttons: outputs must stay OFF.
- [ ] Mill: MDI `M3 S12000` -> OUT1 on; `M4` -> OUT1 + OUT2 (direction); `M5` -> off.
- [ ] Huanyang VFD (if used), spindle load disconnected or spindle free to turn:
  - [ ] VFD powered off but RS485 adapter plugged: Cycle Start must be refused
        (*"Huanyang VFD is not responding"*).
  - [ ] Run M3 -> spindle starts; M5 -> stops; M4 -> runs in reverse.
  - [ ] Pull the RS485 cable while running -> comm-loss alarm -> E-STOP. Reconnect, clear VFD alarm,
        then reset.
- [ ] Laser (BreakoutCut): test pulse refused while door open / chiller no-flow inputs are inactive;
      opening the lid during a laser job pauses it and switches the beam off immediately.

## Phase 8 - With motors (low speed, hand on the hardwired E-stop)

Set all axis max velocities to ~25 % and accelerations to ~50 % for this phase.

- [ ] Jog each axis (continuous + step); releasing the key/button stops smoothly. Alt-Tab away while
      jogging: the axis must stop within ~0.5 s (jog dead-man).
- [ ] With soft limits enabled and the machine homed: continuous jog toward a limit decelerates and
      stops at the limit (no E-stop); a program exceeding the limits is refused at Cycle Start.
- [ ] Real homing on switches (all axes).
- [ ] Air-cut a real job; feed hold / resume / stop mid-job; listen for stutter (it would point at
      the 3-slice flow control or Windows timer jitter).
- [ ] Mark a reference point, run a long program, return with `G53` / go-to-zero: position error
      should be at the step resolution (no lost steps).
- [ ] Restore normal speeds/accelerations, repeat one job.

---

## Results

| Phase | Assumption / area | Result | Notes |
|-------|-------------------|--------|-------|
| 1 | Report rate >= 2 Hz, continuous when idle | | Hz: |
| 2 | Fault flag 0x20 = HW E-stop | | |
| 3 | Fault flag 0x10 = HW limit | | |
| 4 | `live_steps` units & sign | | |
| 5 | 3 slices in flight OK | | |
| 5 | State after `CancelSequence` | | |
| 6 | `latched_steps` = probe trigger | | |
| 7 | VFD retry / comm loss | | |
| 8 | Motion quality with motors | | |
