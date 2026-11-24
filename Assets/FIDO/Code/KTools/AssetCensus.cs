using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using UnityEngine.Profiling;
using ILogger = ReduxLib.Logging.ILogger;

namespace KTools
{
    /// <summary>
    /// GPU-side asset census and forced unload, for the VRAM growth seen after Revert to Launch
    /// (notes/20261123-211500_bug_gpu-hang-after-reverts.md). Player.log showed Unity's
    /// "Unloading N unused Assets" only at startup, never at a revert, so whatever the old flight
    /// created and dropped stays loaded. Not tied to Training: call Take() anywhere (station work,
    /// docking, scene changes) to see what is loaded and what grew since the last census.
    ///
    /// Log lines (Ksp2.log, logger FIDO.Census):
    ///   [Census] &lt;tag&gt;: textures N (X MB, Unity texture memory Y MB), render textures ..., meshes ..., materials ...
    ///   [Census] &lt;tag&gt; grew: name xN +X MB; ...      (top growers by size since the previous census)
    ///   [Census] unload &lt;tag&gt;: X s, textures -N (-X MB), ...
    /// </summary>
    public static class AssetCensus
    {
        static readonly ILogger logger = ReduxLib.ReduxLib.GetLogger("FIDO.Census");

        public class Snapshot
        {
            public string tag;
            public float time;
            // per kind: count, bytes
            public readonly Dictionary<string, (int n, long bytes)> kinds = new();
            // per kind:name group: count, bytes
            public readonly Dictionary<string, (int n, long bytes)> groups = new();
            public long unity_texture_memory;
        }

        static Snapshot last;
        const int TopGrowers = 15;

        /// <summary>Counts textures, render textures, meshes and materials, logs it and the
        /// biggest growth since the previous census. Takes ~0.1-0.5 s with a full flight scene.</summary>
        public static Snapshot Take(string tag, bool log_growth = true)
        {
            var s = new Snapshot { tag = tag, time = Time.realtimeSinceStartup };
            try
            {
                foreach (var t in Resources.FindObjectsOfTypeAll<Texture>())
                    Add(s, t is RenderTexture ? "rendertex" : "texture", t, TextureBytes(t));
                foreach (var m in Resources.FindObjectsOfTypeAll<Mesh>())
                    Add(s, "mesh", m, MeshBytes(m));
                foreach (var m in Resources.FindObjectsOfTypeAll<Material>())
                    Add(s, "material", m, 0);
                try { s.unity_texture_memory = (long)Texture.currentTextureMemory; } catch (Exception) { }

                logger.LogInfo($"[Census] {tag}: {Kind(s, "texture")}, render textures {Kind(s, "rendertex")}, " +
                               $"meshes {Kind(s, "mesh")}, materials {s.kinds.GetValueOrDefault("material").n}, " +
                               $"Unity texture memory {Mb(s.unity_texture_memory)} MB, " +
                               $"mono {Mb(GC.GetTotalMemory(false))} MB, unity {Mb(Profiler.GetTotalAllocatedMemoryLong())} MB");
                if (log_growth && last != null) LogGrowth(last, s);
            }
            catch (Exception e)
            {
                logger.LogError($"[Census] {tag}: {e}");
            }
            last = s;
            return s;
        }

        /// <summary>Unity's Resources.UnloadUnusedAssets after a full GC (so dropped C# references
        /// count as unused). Poll the returned operation; Finish() logs what it freed.</summary>
        public static AsyncOperation StartUnload(string tag)
        {
            try
            {
                GC.Collect();
                GC.WaitForPendingFinalizers();
                GC.Collect();
                logger.LogInfo($"[Census] unload {tag}: start");
                return Resources.UnloadUnusedAssets();
            }
            catch (Exception e)
            {
                logger.LogError($"[Census] unload {tag}: {e}");
                return null;
            }
        }

        /// <summary>Logs an unload's duration and, when census is on, the before/after difference.</summary>
        public static void Finish(string tag, float started, Snapshot before, bool census)
        {
            float secs = Time.realtimeSinceStartup - started;
            if (!census || before == null)
            {
                logger.LogInfo($"[Census] unload {tag}: done in {secs:0.0} s");
                return;
            }
            var after = Take(tag + " after unload", log_growth: false);
            var sb = new StringBuilder($"[Census] unload {tag}: done in {secs:0.0} s, freed");
            foreach (var k in new[] { "texture", "rendertex", "mesh", "material" })
            {
                var b = before.kinds.GetValueOrDefault(k);
                var a = after.kinds.GetValueOrDefault(k);
                sb.Append($" {k} {a.n - b.n:+#;-#;0} ({Mb(a.bytes - b.bytes):+#;-#;0} MB),");
            }
            sb.Append($" Unity texture memory {Mb(after.unity_texture_memory - before.unity_texture_memory):+#;-#;0} MB");
            logger.LogInfo(sb.ToString());
            // what the unload removed, biggest first
            LogChange(before, after, "freed", shrink: true);
        }

        // ================================================================ details

        static void Add(Snapshot s, string kind, UnityEngine.Object o, long bytes)
        {
            var k = s.kinds.GetValueOrDefault(kind);
            s.kinds[kind] = (k.n + 1, k.bytes + bytes);
            string key = kind + ":" + Group(o.name);
            var g = s.groups.GetValueOrDefault(key);
            s.groups[key] = (g.n + 1, g.bytes + bytes);
        }

        // "Kerbin_Tile_123 (Instance)" -> "Kerbin_Tile_#": numbered copies count as one group
        static string Group(string name)
        {
            if (string.IsNullOrEmpty(name)) return "(unnamed)";
            var sb = new StringBuilder(name.Length);
            bool digits = false;
            foreach (char c in name)
            {
                if (char.IsDigit(c)) { if (!digits) sb.Append('#'); digits = true; }
                else { sb.Append(c); digits = false; }
            }
            var g = sb.ToString().Replace(" (Instance)", "");
            return g.Length > 60 ? g.Substring(0, 60) : g;
        }

        static long TextureBytes(Texture t)
        {
            long b = 0;
            try { b = Profiler.GetRuntimeMemorySizeLong(t); } catch (Exception) { }
            if (b > 0) return b;
            // release players can report 0: estimate RGBA32 with mips
            try
            {
                long px = (long)t.width * t.height;
                if (t is RenderTexture rt) px *= Math.Max(1, rt.volumeDepth);
                b = px * 4;
                if (t.mipmapCount > 1) b = b * 4 / 3;
            }
            catch (Exception) { }
            return b;
        }

        static long MeshBytes(Mesh m)
        {
            long b = 0;
            try { b = Profiler.GetRuntimeMemorySizeLong(m); } catch (Exception) { }
            if (b > 0) return b;
            try { b = (long)m.vertexCount * 48; } catch (Exception) { }
            return b;
        }

        static void LogGrowth(Snapshot from, Snapshot to) => LogChange(from, to, "grew", shrink: false);

        static void LogChange(Snapshot from, Snapshot to, string word, bool shrink)
        {
            var rows = new List<(string key, int dn, long db)>();
            foreach (var kv in to.groups.Concat(from.groups.Where(kv => !to.groups.ContainsKey(kv.Key))))
            {
                var a = to.groups.GetValueOrDefault(kv.Key);
                var b = from.groups.GetValueOrDefault(kv.Key);
                long db = a.bytes - b.bytes;
                int dn = a.n - b.n;
                if (shrink ? db < 0 || dn < 0 : db > 0 || dn > 0) rows.Add((kv.Key, dn, db));
            }
            if (rows.Count == 0) { logger.LogInfo($"[Census] {to.tag} {word}: nothing since {from.tag}"); return; }
            var top = (shrink ? rows.OrderBy(r => r.db).ThenBy(r => r.dn) : rows.OrderByDescending(r => r.db).ThenByDescending(r => r.dn))
                .Take(TopGrowers);
            var sb = new StringBuilder($"[Census] {to.tag} {word} since {from.tag}:");
            foreach (var r in top)
                sb.Append($" {r.key} x{r.dn:+#;-#;0} {Mb(r.db):+0.0;-0.0;0} MB;");
            logger.LogInfo(sb.ToString());
        }

        static string Kind(Snapshot s, string kind)
        {
            var k = s.kinds.GetValueOrDefault(kind);
            return $"{k.n} ({Mb(k.bytes):0} MB)";
        }

        static double Mb(long bytes) => bytes / (1024.0 * 1024.0);
    }
}
