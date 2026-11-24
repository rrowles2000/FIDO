using System;
using System.IO;

namespace KTools
{
    // K2D2 -> FIDO rename (2026-11-24): on the first run, copy the old k2d2_* settings, craft memory
    // and exported models to their fido_* names. It looks in this mod's folder and in a sibling
    // mods/K2D2 folder, if one is still there. Files are copied, never moved, so the old ones stay
    // as a backup. A fido_* file that already exists is never overwritten.
    internal static class LegacyFiles
    {
        static readonly (string from, string to)[] Files =
        {
            ("k2d2_settings.json", "fido_settings.json"),
            ("k2d2_landing_atmo.json", "fido_landing_atmo.json"),
            ("k2d2_landing_vac.json", "fido_landing_vac.json"),
            ("k2d2_craft_memory.json", "fido_craft_memory.json"),
            ("k2d2_craft_memory.json.bak", "fido_craft_memory.json.bak"),
        };

        const string ModelsFrom = "k2d2_models", ModelsTo = "fido_models";

        public static void Migrate(string modFolder, Action<string> log)
        {
            try
            {
                var sibling = Path.Combine(Path.GetDirectoryName(modFolder) ?? modFolder, "K2D2");
                foreach (var src in new[] { modFolder, sibling })
                {
                    if (!Directory.Exists(src)) continue;
                    foreach (var (from, to) in Files)
                    {
                        var a = Path.Combine(src, from);
                        var b = Path.Combine(modFolder, to);
                        if (File.Exists(a) && !File.Exists(b))
                        {
                            File.Copy(a, b);
                            log($"[FIDO] copied legacy {a} -> {to}");
                        }
                    }
                    var md = Path.Combine(src, ModelsFrom);
                    if (Directory.Exists(md))
                    {
                        var dest = Path.Combine(modFolder, ModelsTo);
                        Directory.CreateDirectory(dest);
                        int n = 0;
                        foreach (var f in Directory.GetFiles(md))
                        {
                            var b = Path.Combine(dest, Path.GetFileName(f));
                            if (File.Exists(b)) continue;
                            File.Copy(f, b);
                            n++;
                        }
                        if (n > 0) log($"[FIDO] copied {n} legacy model file(s) from {md}");
                    }
                }
            }
            catch (Exception e)
            {
                log($"[FIDO] legacy file copy failed: {e.Message}");
            }
        }
    }
}
