# FIDO — Flight Dynamics Operator (formerly K2-D2, Redux port)

> **Renamed 2026-11-24.** This fork of K2D2Redux is now **FIDO** (Flight Dynamics Operator, after NASA's
> Flight Dynamics Officer console): mod_id `FIDO`, `Fido.dll`, namespace `Fido.*`, game folder `mods/FIDO`.
> To move an existing install and its saves over, close the game and run `Tools/migrate_install.py`
> (it backs up first), then `Tools/build.sh --install`. FIDO is based on **K2-D2** by Christophe Floutier and
> **K2D2Redux** by IanMealworm / KSP2Community; their credits and the CC BY-SA 4.0 licence are kept below
> and in [`NOTICE.md`](NOTICE.md). The history below still says K2-D2 where it describes the earlier work.
>
> Not renamed (the prebuilt UI bundle can't be rebuilt without the Unity editor): the `K2UI` control
> namespace, `Assets/UI/K2D2_UI/K2D2_Window.uxml` and its addressables groups, USS class names, and the
> `K2D2.LateBound` carrier marker.


An astromech-style autopilot suite for Kerbal Space Program 2, ported to the **Redux** modding framework from [Christophe Floutier's original K2-D2](https://github.com/cfloutier) (built for SpaceWarp1).

K2-D2 gives you one panel (`Alt-O` or the AppBar icon) with a set of autopilots:

- **Node** - executes the next maneuver node, with auto-circularize at Ap/Pe and one-click buttons to create a node at the next apoapsis or periapsis
- **Lift** - automated ascent guidance with a configurable altitude/heading profile, an optional roll program, and automatic circularization at the end of the climb
- **Landing** - automated descent, braking, and touchdown, with an optional Precision Landing mode (currently bodies with no atmosphere only) that targets a specific site via Redux's waypoint system
- **Docking** - automated final approach and docking
- **Attitude** - point-and-hold attitude control (a simple plane autopilot)

## Status

This is a from-scratch port of the original code onto Redux's APIs, not a compatibility shim - it required chasing down and fixing a number of real differences between the old SpaceWarp1/UitkForKsp2 environment and Redux (custom UI controls, orbit representation, and more). The full blow-by-blow of everything found and fixed is in [`NOTICE.md`](NOTICE.md), kept as a development log for anyone porting a similar mod and hitting the same walls.

**Confirmed working (tested in-game):**
- Full UI - all tabs, styling, custom controls
- Node autopilot
- Lift/ascent autopilot - flies the configured profile to orbit, including an optional roll program, then creates and flies its own circularization node
- Landing autopilot - descent, braking, and touchdown, including collision detection and precision landing on bodies with no atmosphere
- Docking autopilot - final approach and main-thrust kill-speed/brake
- Attitude hold
- Auto-staging, with a player-facing on/off toggle in the window's title bar

**Known limitations:**
- Precision Landing on atmospheric bodies is still a work in progress and isn't exposed in the UI yet - only bodies with no atmosphere have a player-facing Precision Landing option for now.

## Installation

1. Install [Redux](https://ksp2redux.org) for Kerbal Space Program 2.
2. Make sure you're on KSP2 beta snapshot 26w33a or newer (Redux Launcher: Settings [gear icon to the right of the "Mods" tab] > Release Channel > Beta, then update to the latest beta).
3. Download the latest K2-D2 release and drop the contents into your KSP2 `mods` folder (merge folders if prompted).

## Credits

- **[Christophe Floutier](https://github.com/cfloutier)** - original K2-D2 mod for SpaceWarp1
- **IanMealworm** - Redux port
- **[Mole](https://github.com/Mole1803)** - original Circularize work
- **[schlosrat](https://forum.kerbalspaceprogram.com/index.php?/profile/141963-schlosrat/)** - original testing and code help, especially node creation
- **Opus** - named the mod
- **[cheese3660](https://github.com/cheese3660)** - [SpaceWarp](https://github.com/Halbann) and [AutoBurn](https://github.com/cheese3660/AutoBurn), which the original mod was built on
- **[Halbann](https://github.com/Halbann)** - [LazyOrbit](https://github.com/Halbann/LazyOrbit), which the original mod's first steps were based on
- **[KSP2Community](https://github.com/KSP2Community)** - the Redux modding framework and the `Redux.Template` project scaffold this port is built on

## Changelog

See [`CHANGELOG.md`](CHANGELOG.md) for release notes.

## License

CC BY-SA 4.0 - see [`LICENSE.md`](LICENSE.md). Same license as the original K2-D2, inherited as required by its ShareAlike terms.
