---
id: 20261123-211500
category: bug
title: GPU hang (NVIDIA Xid 109) on the 3rd Training flight, after 2 reverts to launch
status: open (measured x2 2026-11-24: small VRAM growth over flights 1-3, load peaks 11.2-11.6 of 12.3 GB)
created: 2026-11-23 21:15:00
---

# GPU hang after repeated Revert to Launch

**What happened:** Training session of 5 flights on R2-1 (83 t). Flights 1 and 2 reached orbit
(`[Next] RESULT OK`, 21:01:05 and 21:06:44), each followed by the Training tab's Revert to Launch.
Flight 3 launched at 21:06:59; at T+4 s the game froze (PerfLog's last second: 1.2 fps, 21:07:09) and the
user had to kill the window. No exception in Ksp2.log or Player.log; Unity wrote no crash report.

**Kernel log (journalctl -k):**

    21:07:08 NVRM: Xid (PCI:0000:01:00): 109, pid=61042, name=Unity Main Thre, channel 0x00000035,
             errorString CTX SWITCH TIMEOUT, Info 0xac040

**System:** RTX 3060 12 GB, driver 580.178.04, Proton Experimental (DXVK, D3D11),
Redux stable 0.2.8.5.103184 / Unity 6000.4.1f1.

**Likely the same crash as before:** the 2026-11-14 handoff notes a d3d11/DXVK crash after ~4 reverts in
one session (the Training cap of 3 flights existed because of it; it was removed in 2be8370). This time it
came after 2 reverts.

**Suspected cause: GPU memory (VRAM) leak per revert** in the stable build. Redux snapshot 0.2.9.0.108007
(2026-11-01) lists "fixes bugs across flight, construction and rendering, including some severe VRAM leaks".
PerfLog only sees CPU-side memory: over the two reverts Unity allocated memory went ~3,880 -> 4,130 MB and
managed (mono) ~790 -> 1,135 MB (session `logs/perf/perf-20261123-205323.csv`), consistent with
accumulation but VRAM itself is not measured.

Not thought to be today's changes: PerfLog only reads counters (discrete test was `off`), and the K2D2
reset button is VAB-only.

## 2nd occurrence: 2026-11-24 (Unity crash in d3d11)

Training session of 5 flights on R2-1 again. Flight 1 OK (15:40:29), revert; flight 2 OK (15:44:49), revert;
loading into flight 3 at 15:44:58, the vessel spawned and SAS came on, then at **15:44:59 Unity's crash
handler fired (`Crash!!!`)**. The native stack is entirely inside `d3d11` (DXVK), with no managed or mod frames.
This time no Xid in the kernel log: D3D11 failed, the driver didn't hang. Crash report with minidump:
`compatdata/954850/.../Temp/Intercept Games/Kerbal Space Program 2/Crashes/Crash_2026-11-24_154459854/`
(crash.dmp, Player.log). Game log archived as `logs/Ksp2-20261124-153452.log`, PerfLog
`perf-20261124-153500.csv`.

Same point as yesterday: the 3rd flight, after 2 reverts. CPU-side memory does **not** climb per revert:
Unity allocated ~4.01 GB flight 1, ~4.14 GB flight 2, 4.14 GB at the crash; mono swings 720-1,090 MB with
GC. So the leak, if it is one, is not in managed/Unity CPU memory, and VRAM stays the main suspect.

**Step 1 done (2026-11-24):** the live view server now samples nvidia-smi every 5 s while KSP2_x64.exe runs
(`logs/perf/vram-YYYYMMDD.csv`), joins it onto the PerfLog series and charts VRAM / Unity / Mono on the
Performance tab ("Memory over the session"), with a Peak VRAM tile. Next Training session will show whether
VRAM climbs per revert.

## Measured session: 2026-11-24 15:51-16:20 (5/5 flights, 4 reverts, no crash)

`logs/perf/vram-20261124.csv` (337 samples, 5 s), PerfLog `perf-20261124-155152.csv`, log
`logs/Ksp2-20261124-155146.log`. VRAM is total GPU use (the desktop adds ~0.5-0.7 GB: Xorg, gnome-shell, Firefox).

| | VRAM |
|---|---|
| Steady in flight, every flight 1-5 | 9.3-9.8 GB, flat - **no climb per revert** |
| In the MainMenu step of a revert | 7.0 / 8.4 / 9.9 / 9.7 GB (not fully released before the reload) |
| Loading the flight scene (VAB->flight, each revert) | **11.1-11.4 GB** peaks: 11,283 / 11,373 / 11,097 / 11,145 / 11,273 MB |
| Card | 12,288 MB |

So it is **not a leak**: each scene load briefly needs ~1.5-2 GB above steady state, which puts the peak within
~0.9 GB of the card's limit (and a 5 s sample understates the true peak). The crashes are the loads where the
peak goes over: the old scene's VRAM isn't freed yet when the new one is loaded (MainMenu values rise over
the session), so whether a given revert crashes depends on timing. Two crashes on the 3rd flight look like chance.

Graphics settings now (Global/Settings.json): QualityPreset high, **TextureQuality 0 (full-size textures)**,
2560x1440, CloudQuality 2, GroundShadingQuality 2, WaterQuality 2, Shadows All/High.

## Second measured session: 2026-11-24 18:16-18:39 (same settings, 5/5 OK, no crash)

Per flight (5 s samples; in-flight median / scene-load peak, MB):

| Flight | 15:51 session | 18:16 session |
|---|---|---|
| 1 | 9,452 / 11,283 | 9,434 / 11,213 |
| 2 | 9,619 / 11,373 | 9,618 / 11,285 |
| 3 | 9,599 / 11,097 | 9,696 / 11,539 |
| 4 | 9,388 / 11,145 | 9,696 / 11,559 |
| 5 | 9,533 / 11,273 | 9,560 / 11,538 |

The user read the Memory chart as a leak. The numbers: in the 18:16 session VRAM does grow, by ~260 MB in flight and
~350 MB at the load peak over flights 1-3, then levels off (flight 5 lower). The 15:51 session grew ~150 MB over
flights 1-2 and then fell back. Unity CPU memory grows steadily ~15-17 MB per flight in both (4,039 -> 4,106 MB).
So: **a small leak per revert is plausible** (a few hundred MB over the first reverts), but the crash margin is
mostly the ~1.9 GB load spike on top of ~9.5 GB steady. Headroom at the worst load was ~0.7 GB (11,559 of 12,288),
and the 5 s samples miss the real top. From 18:45 the sampler runs every 1 s.

## 4th occurrence: 2026-11-24 19:41 (stall, polar Training)

Flights 1 and 2 OK; after the 2nd revert (19:41:46) the game stalled loading flight 3 - the log stops there. No Xid
in the kernel log, and VRAM was only ~6.7 GB (1 s samples, `vram-20261124.csv`). Same point as every time (3rd flight
after 2 reverts), but not near the VRAM limit - so not only VRAM exhaustion. Texture Quality still 0.

## Polar Training session 2026-11-24 19:48 (crash loading flight 3)

1 s samples. Flight 1 settles 9.36-9.43 GB like an equatorial flight 1; after the revert flight 2 settles
10.03-10.36 GB: **+0.65-0.9 GB per revert vs +0.2-0.3 GB for equatorial runs**. The 19:37 east flight that
followed a polar flight also started at 10.15 GB, so the previous flight's retained VRAM drives it. Loading
flight 3 (revert 20:00:16) reached 11.56 GB; Ksp2.log stops 20:00:23.9; Unity crash report
`Crash_2026-11-24_200024201` (d3d11 stack).

**Player.log finding:** Unity's `Unloading N unused Assets ... Loaded Objects now: X` appears only 3 times at
startup (last ~471k objects) and **never at a Revert to Launch**, in every session. Hypothesis: whatever the old
flight created and dropped (terrain tiles, meshes, render textures) stays loaded; polar tracks cover terrain the
equatorial runs never touch, so they leave more behind.

**Test in place (FIDO, 2026-11-24):** Training runs GC + `Resources.UnloadUnusedAssets()` once back on the pad
after each revert, and `KTools.AssetCensus` logs `[Census]` lines (counts/sizes by type, top growers, what the
unload freed). Both toggles on the Training tab. Reading the result: VRAM back to ~9.4 GB after the unload =
confirmed and fixed; objects freed but VRAM still high = DXVK keeping memory (try `DXVK_HUD=memory`); nothing
freed = native graphics buffers the game never releases (Redux snapshot route).

## Result 2026-11-24 20:12-20:41: forced unload after revert (FIDO 7a2bff7), 5 polar flights

5/5 flown, 5/5 orbit, game quit normally (no crash report, log ends after flight 5). Before the fix this
setup crashed loading flight 3 every time.

| Flight | Settled on pad (GB) | End of flight (GB) | Load peak before it (GB) |
|---|---|---|---|
| 1 | 9.66 | 9.89 | 11.58 (first load) |
| 2 | 9.66 | 9.83 | 11.78 |
| 3 | 9.38 | 9.57 | 11.78 |
| 4 | 9.38 | 9.53 | 11.83 |
| 5 | 9.38-9.43 | 9.45-9.72 | 11.72 |

- **Settled level no longer creeps.** Before the fix polar flight 2 settled 10.03-10.36 GB; now 9.66 then 9.38.
- Player.log now shows "Unloading ~1865 unused Assets" at each revert (4 times, Loaded Objects +~450 per revert).
- Unload takes 0.5-0.6 s. Census: frees ~1260 materials (ReentryShockwave x642 each time), ~57 meshes
  (ProceduralFairing), 2 kerbin_scaledclouds textures (37 MB). "Unity texture memory" stays 4094 MB. The
  VRAM drop is bigger than what the census counts, so most of it is GPU memory behind objects the census
  does not size (mesh/graphics buffers, materials).
- **Still open: the load peak.** The unload runs after the new scene is on the pad, so the overlap peak at
  each revert is still 11.7-11.8 of 12.0 GB (higher than the 11.56 GB of the 20:00 crash). About 0.4 GB
  headroom; a bigger scene could still tip it. Options: unload before the revert too (old scene, then
  revert), the two-step revert, or Texture Quality 1.
- The 19:41 stall at 6.7 GB is not explained by this.
- Flight checks: heading 355.6 deg on every flight; plane node predicts 89.97 deg, flights end 90.18-90.22 deg;
  stage 1 dropped at Pe 3.8-49 km (all come down); Training "Finished: 5/5 flown, 5 reached orbit".

## 2026-11-24 20:47-20:55: unload BEFORE revert (FIDO 0432be3) - no effect on the peak

- 1 polar flight to orbit + revert + part of flight 2 (user quit from ESC menu at T+29 s; no crash).
- "unload before revert 1" freed texture 0 MB, rendertex 0, mesh 3, material 703 - nothing on the GPU.
- Revert load peak 11.76 GB at 20:54:32 (unchanged vs 11.7-11.8); settled 9.5-9.8 GB.
- **Not overlap:** VRAM fell to 7.2 GB at 20:54:26 (old scene released) before the new load pushed it to
  11.76. The first load from the main menu (from 1.2 GB) also peaked 11.58 GB. A flight-scene load
  transiently needs ~2 GB above its settled level whatever came before - unloading cannot lower the peak.
- Next: turn the before-revert toggle off; try Texture Quality one step lower and check the load peak.

## To do (later)

- **Big scenes, not only reverts:** the same retention may build up elsewhere - large space stations, docking,
  many vessels in physics range, long sessions with scene changes. Plan a test: take an asset census + VRAM
  before/after loading a big station and after switching away, and see whether a manual unload frees it.
  (AssetCensus.Take can be called from anywhere; a census/unload button outside Training would make this easy.)

## To do (revised)

- **Fix: cut VRAM headroom.** Set Texture Quality to half (TextureQuality 1) in the game's graphics settings
  first - typically saves several GB on full-res textures; then clouds/shadow resolution if needed. Re-run a
  5-flight Training and check that load peaks are under ~10 GB.
- (done 18:45) sample every 1 s to see the real load peak.
- To settle the leak: one longer Training (8-10 flights) at 1 s sampling - a leak keeps climbing, a plateau does not.
- The Redux snapshot's "VRAM leak fixes" are less relevant now (no leak seen in flight); keep as a fallback.

## Original to do

1. **(done 2026-11-24) Measure VRAM:** have the live view server (tools/liveview) sample `nvidia-smi
   --query-gpu=memory.used` every ~5 s while the game runs, write it next to the PerfLog CSV, and show it
   on the Performance tab with revert markers. Confirm whether VRAM climbs per revert and the hang comes
   near 12 GB.
2. **If it is a leak:** try the latest Redux snapshot (VRAM leak fixes, 0.2.9.0.108007+) - needs K2D2 and
   the other mods rebuilt against it. Otherwise look for what survives a revert (render textures, cameras,
   K2D2/PerfLog objects kept across scenes).
3. **Workaround meanwhile:** restart the game after every 2 Training flights; consider bringing back a
   Training cap (default 2) or a "restart the game" prompt in the Training tab.
