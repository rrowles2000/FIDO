#!/usr/bin/env python3
"""One-time K2D2 -> FIDO migration of the game install and the save files (2026-11-24).

Run with the game closed, then run `Tools/build.sh --install`.

1. Backs up mods/K2D2 and the Saves folder to <backup-dir>/k2d2-to-fido-<time>/ (tar.gz).
2. Moves mods/K2D2 to mods/FIDO, removes K2D2.dll and the old k2d2_* patches/csv (in the backup),
   renames k2d2_settings/landing/craft_memory files and k2d2_models/ to fido_*.
3. Points the addressables catalog at {SpaceWarpPaths.FIDO} (SpaceWarp names the path variable
   after the mod_id). The bundles themselves are unchanged.
4. Rewrites the saved part-module types in every save (.json) that mentions K2D2:
   K2D2.Mission.Module_K2Mission, K2D2 -> Fido.Mission.Module_FidoMission, Fido (and the Data_ /
   PartComponentModule_ / ActionTarget types), plus the save's mod list entry K2D2 -> FIDO.
   .meta files (the mod list) get the same rewrite. Each rewritten file is checked to still parse as JSON before it is written (tmp -> replace).

  --dry-run   report what would change, touch nothing
"""
import argparse, datetime, json, os, re, shutil, subprocess, sys, tarfile

HOME = os.path.expanduser("~")
GAME = os.environ.get("KSP2", os.path.join(HOME, ".steam/steam/steamapps/common/Kerbal Space Program 2"))
SAVES = os.path.join(HOME, ".steam/debian-installation/steamapps/compatdata/954850/pfx/drive_c/users/steamuser/"
                           "AppData/LocalLow/Intercept Games/Kerbal Space Program 2/Saves")

RENAMES = [("k2d2_settings.json", "fido_settings.json"), ("k2d2_landing_atmo.json", "fido_landing_atmo.json"),
           ("k2d2_landing_vac.json", "fido_landing_vac.json"), ("k2d2_craft_memory.json", "fido_craft_memory.json"),
           ("k2d2_craft_memory.json.bak", "fido_craft_memory.json.bak"), ("k2d2_models", "fido_models")]
REMOVE = ["K2D2.dll", "patches/k2d2_mission.lua", "patches/k2d2_actions.lua", "localizations/k2d2_mission.csv"]

SAVE_SUBS = [
    (re.compile(rb"\bK2D2\.(?=[A-Za-z])"), b"Fido."),               # namespaces K2D2.Mission. -> Fido.Mission.
    (re.compile(rb"_K2Mission\b"), b"_FidoMission"),
    (re.compile(rb"_K2ActionTarget\b"), b"_FidoActionTarget"),
    (re.compile(rb", K2D2(?=[,\"])"), b", Fido"),                   # assembly name in type names
    (re.compile(rb'("ModId"\s*:\s*)"K2D2"'), rb'\1"FIDO"'),
    (re.compile(rb'("ModName"\s*:\s*)"K2-D2"'), rb'\1"FIDO"'),
]


def game_running():
    out = subprocess.run(["ps", "-eo", "args"], capture_output=True, text=True).stdout
    return any("ksp2_x64.exe" in l.lower() for l in out.splitlines())


def migrate_save(path, dry):
    raw = open(path, "rb").read()
    if b"K2D2" not in raw and b"K2-D2" not in raw:
        return 0
    new, n = raw, 0
    for rx, rep in SAVE_SUBS:
        new, k = rx.subn(rep, new)
        n += k
    if new == raw:
        return 0
    json.loads(new.decode("utf-8-sig"))  # must still parse
    left = len(re.findall(rb"K2D2", new))
    if left:
        print(f"  note: {left} K2D2 mention(s) left in {os.path.basename(path)}")
    if not dry:
        tmp = path + ".fido-tmp"
        with open(tmp, "wb") as f:
            f.write(new)
        os.replace(tmp, path)
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--game", default=GAME)
    ap.add_argument("--saves", default=SAVES)
    ap.add_argument("--backup-dir", default=os.path.join(HOME, "Projects/KSP/ksp2-redux/backups"))
    a = ap.parse_args()
    if game_running():
        sys.exit("KSP2_x64.exe is running: close the game first")

    old, new = os.path.join(a.game, "mods", "K2D2"), os.path.join(a.game, "mods", "FIDO")
    have_old = os.path.isdir(old)
    if not have_old and not os.path.isdir(new):
        sys.exit(f"neither {old} nor {new} exists")
    dry = a.dry_run

    # 1. backup
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    bdir = os.path.join(a.backup_dir, f"k2d2-to-fido-{stamp}")
    if not dry:
        os.makedirs(bdir, exist_ok=True)
        if have_old:
            with tarfile.open(os.path.join(bdir, "mods-K2D2.tar.gz"), "w:gz") as t:
                t.add(old, arcname="K2D2")
        if os.path.isdir(a.saves):
            with tarfile.open(os.path.join(bdir, "Saves.tar.gz"), "w:gz") as t:
                t.add(a.saves, arcname="Saves")
        print(f"backup: {bdir}")

    # 2. install folder
    if have_old:
        print(f"move {old} -> {new}")
        if not dry:
            if os.path.isdir(new):
                sys.exit(f"{new} already exists next to {old}: sort that out by hand")
            shutil.move(old, new)
    root = new if not dry else (old if have_old else new)
    for rel in REMOVE:
        p = os.path.join(root, rel)
        if os.path.exists(p):
            print(f"remove {rel}")
            if not dry:
                os.remove(p)
    for a_name, b_name in RENAMES:
        p, q = os.path.join(root, a_name), os.path.join(root, b_name)
        if os.path.exists(p) and not os.path.exists(q):
            print(f"rename {a_name} -> {b_name}")
            if not dry:
                os.rename(p, q)

    # 3. addressables catalog
    cat = os.path.join(root, "addressables", "catalog.json")
    if os.path.isfile(cat):
        s = open(cat, encoding="utf-8").read()
        n = s.count("{SpaceWarpPaths.K2D2}")
        if n:
            print(f"catalog: {n} path(s) -> {{SpaceWarpPaths.FIDO}}")
            if not dry:
                json.loads(s.replace("{SpaceWarpPaths.K2D2}", "{SpaceWarpPaths.FIDO}"))
                with open(cat + ".fido-tmp", "w", encoding="utf-8") as f:
                    f.write(s.replace("{SpaceWarpPaths.K2D2}", "{SpaceWarpPaths.FIDO}"))
                os.replace(cat + ".fido-tmp", cat)

    # 4. saves
    files = changes = 0
    for dp, _, fs in os.walk(a.saves):
        for f in fs:
            if f.endswith((".json", ".meta")):
                n = migrate_save(os.path.join(dp, f), dry)
                if n:
                    files += 1
                    changes += n
    print(f"saves: {changes} replacement(s) in {files} file(s){' (dry run)' if dry else ''}")
    if not dry:
        print("next: Tools/build.sh --install")


if __name__ == "__main__":
    main()
