# Lift legacy archive (2026-11-15)

Snapshot of the K2D2 lift code taken just before the Classic and Optimiser lift systems and the
New system's pre-behaviour-model path were removed. Base commit: 8ac68e8 plus the uncommitted
2026-11-15 changes (stage-1 reserve fix, SaveTidy). Not compiled - build.sh only reads
Assets/K2D2/Code.

Removed:
- Classic system: Pilots/Lift/Classic/{AscentClassic,FinalClassic}.cs (CoastingClassic.cs lives on
  as Controlers/Coasting.cs - the New system uses it)
- Optimiser system: Controlers/{Ascent,Coasting}.cs, Stage1/Stage1UI.cs, LiftAscentPath.cs,
  Stage1Sim.cs's SimParams / Run(SimParams) / Stage1Optimiser and the guidance helpers only it used,
  FinalCircularize's node-based burn
- New system's old path: GravityTurnPolicy, GuidancePolicy, PlanLearner.cs, LaunchCheck's
  kick/hold/pull-down grid search, the shield's AoA limit / max-Q throttle-back / drift protection,
  CraftMemory's max-Q hold and plan history, the kick / behaviour-model toggles
- Settings: lift.mode, start_altitude_km, mid/end_rotate_ratio, max_throttle, disposal_*,
  optimise_throttle, roll_program*, next.kick_*, next.behaviour_model

k2d2_craft_memory.json is the game's memory file as it was, including PlanLearner's "plans" and
"qHold" fields, which the new code no longer writes.

To look at or restore anything: the files here keep their original paths under Assets/, so
`diff -r Archive/lift-legacy-2026-11-15/Assets Assets` shows exactly what changed.
