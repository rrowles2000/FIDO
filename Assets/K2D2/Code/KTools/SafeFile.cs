using System;
using System.IO;

namespace KTools
{
    /// <summary>
    /// Crash-safe file writes for data the game can't rebuild (learned craft memory, settings).
    /// File.WriteAllText truncates the file first, so a crash or a full disk part-way through
    /// leaves it empty or cut short, and the next load starts from nothing (review finding C-3).
    /// </summary>
    public static class SafeFile
    {
        /// <summary>
        /// Writes <paramref name="text"/> to path.tmp, then swaps it in for <paramref name="path"/>,
        /// keeping the previous file as path.bak (when <paramref name="keep_backup"/>). At every
        /// moment either the old or the new file is complete on disk.
        /// </summary>
        public static void Write(string path, string text, bool keep_backup = true)
        {
            string tmp = path + ".tmp", bak = path + ".bak";
            File.WriteAllText(tmp, text);
            if (!File.Exists(path))
            {
                File.Move(tmp, path);
                return;
            }
            try
            {
                File.Replace(tmp, path, keep_backup ? bak : null);
            }
            catch (Exception)
            {
                // File.Replace isn't supported everywhere (some Mono/filesystem combinations):
                // fall back to copy-aside, delete, move - the .bak (or .tmp) is complete throughout
                if (keep_backup) File.Copy(path, bak, true);
                File.Delete(path);
                File.Move(tmp, path);
            }
        }

        /// <summary>
        /// Reads <paramref name="path"/> and parses it with <paramref name="parse"/>; if the file is
        /// missing, unreadable or fails to parse, tries path.bak. A file that fails to parse is copied
        /// to path.corrupt-&lt;time&gt; first, so the next save can never destroy the only copy.
        /// Returns false (and default) when neither file gave a value.
        /// </summary>
        public static bool TryRead<T>(string path, Func<string, T> parse, out T value, Action<string> warn = null)
        {
            foreach (var p in new[] { path, path + ".bak" })
            {
                if (!File.Exists(p)) continue;
                try
                {
                    value = parse(File.ReadAllText(p));
                    if (value != null)
                    {
                        if (p != path) warn?.Invoke($"{path} was unreadable - loaded the backup {p}");
                        return true;
                    }
                }
                catch (Exception e)
                {
                    warn?.Invoke($"couldn't read {p}: {e.Message}");
                }
                if (p == path)
                {
                    try
                    {
                        string keep = $"{path}.corrupt-{DateTime.Now:yyyyMMdd-HHmmss}";
                        File.Copy(path, keep, true);
                        warn?.Invoke($"kept the unreadable file as {keep}");
                    }
                    catch (Exception) { }
                }
            }
            value = default;
            return false;
        }
    }
}
