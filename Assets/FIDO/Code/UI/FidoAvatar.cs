using System;
using System.Collections.Generic;
using KTools;
using UnityEngine;
using UnityEngine.UIElements;

namespace Fido.UI
{
    /// <summary>The FIDO Tile icon textures (art/fido-icon/fido-B-tile.svg rendered to PNG), loaded once.</summary>
    public static class FidoIcons
    {
        static readonly Dictionary<string, Texture2D> cache = new Dictionary<string, Texture2D>();

        static Texture2D Get(string file)
        {
            if (cache.TryGetValue(file, out var t) && t != null) return t;
            try { t = AssetsLoader.LoadIcon(file); }
            catch (Exception e) { FidoPlugin.logger.LogWarning($"[FIDO.UI] icon {file}: {e.Message}"); t = null; }
            cache[file] = t;
            return t;
        }

        /// <summary>48 px: AppBar buttons, window headers.</summary>
        public static Texture2D Icon => Get("fido_icon.png") ?? Get("icon.png");
        /// <summary>128 px: the avatar and the About tab.</summary>
        public static Texture2D Avatar => Get("fido_avatar.png") ?? Icon;
    }

    /// <summary>
    /// FIDO's state on the Tile face: an overlay drawn over a host element (the window header icon) with
    /// Painter2D. Idle: nothing. Running: a slow-breathing blue ring. Warning: amber ring. Error: red ring,
    /// faster. Speaking (a new status message): the lens flashes for a moment. The state comes from a
    /// poll function, checked every 150 ms.
    /// </summary>
    public class FidoAvatar : VisualElement
    {
        public enum State { Idle, Running, Warning, Error }

        public struct Reading
        {
            public State state;
            public string message;   // the status line FIDO is showing; a change makes the lens flash
            public string tooltip;
        }

        readonly Func<Reading> poll;
        State state = State.Idle;
        string last_message;
        float flash_until;

        public FidoAvatar(Func<Reading> poll)
        {
            this.poll = poll;
            name = "fido-avatar";
            pickingMode = PickingMode.Ignore;
            style.position = Position.Absolute;
            style.left = style.top = style.right = style.bottom = 0;
            generateVisualContent += Draw;
            schedule.Execute(Tick).Every(150);
        }

        /// <summary>Puts the Tile on the host (as its background) and lays the overlay over it.</summary>
        public static FidoAvatar Attach(VisualElement host, Func<Reading> poll)
        {
            var tex = FidoIcons.Avatar;
            if (tex != null)
            {
                host.style.backgroundImage = new StyleBackground(tex);
                host.style.unityBackgroundImageTintColor = Color.white;
            }
            var a = new FidoAvatar(poll);
            host.Add(a);
            host.pickingMode = PickingMode.Position;   // so its tooltip shows
            return a;
        }

        void Tick()
        {
            Reading r;
            try { r = poll(); }
            catch { return; }
            if (r.message != last_message)
            {
                if (!string.IsNullOrEmpty(r.message) && last_message != null) flash_until = Time.unscaledTime + 0.6f;
                last_message = r.message;
            }
            state = r.state;
            if (parent != null && parent.tooltip != r.tooltip) parent.tooltip = r.tooltip;
            MarkDirtyRepaint();   // animated (breathing ring, flash)
        }

        void Draw(MeshGenerationContext ctx)
        {
            var r = contentRect;
            float size = Mathf.Min(r.width, r.height);
            if (size < 6) return;
            var p = ctx.painter2D;
            var c = r.center;
            float t = Time.unscaledTime;

            // lens flash (lens centre is at 64,70 of the 128 grid, radius 12)
            float flash = flash_until - t;
            if (flash > 0)
            {
                var lens = new Vector2(r.xMin + r.width * 0.5f, r.yMin + r.height * (70f / 128f));
                var glow = FidoTheme.Lens;
                glow.a = Mathf.Clamp01(flash / 0.6f) * 0.85f;
                p.fillColor = glow;
                p.BeginPath();
                p.Arc(lens, size * (16f / 128f), 0, 360);
                p.Fill();
            }

            if (state == State.Idle) return;
            Color ring = state == State.Running ? FidoTheme.Accent : state == State.Warning ? FidoTheme.Warn : FidoTheme.Bad;
            float speed = state == State.Error ? 6f : 2.5f;
            ring.a = state == State.Warning ? 1f : 0.55f + 0.45f * (0.5f + 0.5f * Mathf.Sin(t * speed));
            p.strokeColor = ring;
            p.lineWidth = Mathf.Max(1.5f, size * 0.09f);
            p.BeginPath();
            p.Arc(c, size * 0.5f - p.lineWidth * 0.5f, 0, 360);
            p.Stroke();
        }
    }
}
