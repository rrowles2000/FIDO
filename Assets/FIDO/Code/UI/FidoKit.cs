using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

namespace Fido.UI
{
    /// <summary>
    /// Tones for status colours. Red (Bad) only ever means a problem; Run is "FIDO is flying".
    /// </summary>
    public enum Tone { Muted, Normal, Run, Ok, Warn, Bad }

    /// <summary>
    /// FIDO's small UI kit (code-only, inline styles from the generated FidoTheme tokens), in the style of
    /// the live view: neutral panels, uppercase muted section titles, readout tiles, status pills, an event
    /// list and a thin line chart. Used by the code-built tabs (Mission, Training; later Fleet).
    /// </summary>
    public static class FidoKit
    {
        public static Color ToneColor(Tone t) => t switch
        {
            Tone.Muted => FidoTheme.Muted,
            Tone.Run => FidoTheme.Accent,
            Tone.Ok => FidoTheme.Ok,
            Tone.Warn => FidoTheme.Warn,
            Tone.Bad => FidoTheme.Bad,
            _ => FidoTheme.Fg,
        };

        static void Pad(IStyle s, float v, float h)
        {
            s.paddingTop = s.paddingBottom = v;
            s.paddingLeft = s.paddingRight = h;
        }

        static void Border(IStyle s, Color c, float width, float radius)
        {
            s.borderTopColor = s.borderBottomColor = s.borderLeftColor = s.borderRightColor = c;
            s.borderTopWidth = s.borderBottomWidth = s.borderLeftWidth = s.borderRightWidth = width;
            s.borderTopLeftRadius = s.borderTopRightRadius = s.borderBottomLeftRadius = s.borderBottomRightRadius = radius;
        }

        /// <summary>A panel: panel colour, 1 px line border, rounded; optional section title.</summary>
        public static VisualElement Panel(VisualElement parent, string title = null)
        {
            var p = new VisualElement();
            p.AddToClassList("fido-panel");
            p.style.backgroundColor = FidoTheme.Panel;
            Border(p.style, FidoTheme.Line, 1, 6);
            Pad(p.style, 6, 8);
            p.style.marginBottom = 6;
            if (title != null) p.Add(SectionTitle(title));
            parent?.Add(p);
            return p;
        }

        public static Label SectionTitle(string text)
        {
            var l = new Label(text.ToUpperInvariant());
            l.AddToClassList("fido-section-title");
            l.style.color = FidoTheme.Muted;
            l.style.fontSize = 11;
            l.style.unityFontStyleAndWeight = FontStyle.Bold;
            l.style.letterSpacing = 1;
            l.style.marginBottom = 4;
            l.style.marginLeft = l.style.marginRight = l.style.marginTop = 0;
            Pad(l.style, 0, 0);
            return l;
        }

        /// <summary>Plain body text in the foreground colour (bold for headings), wrapping.</summary>
        public static Label Text(VisualElement parent, string text = "", bool bold = false)
        {
            var l = new Label(text);
            l.style.color = FidoTheme.Fg;
            l.style.whiteSpace = WhiteSpace.Normal;
            if (bold) l.style.unityFontStyleAndWeight = FontStyle.Bold;
            l.style.marginLeft = l.style.marginRight = 0;
            Pad(l.style, 1, 0);
            parent?.Add(l);
            return l;
        }

        /// <summary>Muted, smaller, wrapping explanatory text.</summary>
        public static Label Hint(VisualElement parent, string text = "")
        {
            var l = Text(parent, text);
            l.style.color = FidoTheme.Muted;
            l.style.fontSize = 12;
            return l;
        }

        public static VisualElement Row(VisualElement parent, float gap = 4)
        {
            var r = new VisualElement();
            r.style.flexDirection = FlexDirection.Row;
            r.style.flexWrap = Wrap.Wrap;
            r.style.marginBottom = gap;
            parent?.Add(r);
            return r;
        }

        public static void SetTone(Label l, Tone t) => l.style.color = ToneColor(t);
    }

    /// <summary>A readout tile: small muted caption over a bold value.</summary>
    public class Readout : VisualElement
    {
        readonly Label caption, value;

        public Readout(string caption_text, VisualElement parent = null)
        {
            AddToClassList("fido-readout");
            style.backgroundColor = FidoTheme.Bg;
            style.borderTopLeftRadius = style.borderTopRightRadius = style.borderBottomLeftRadius = style.borderBottomRightRadius = 4;
            style.paddingTop = style.paddingBottom = 3;
            style.paddingLeft = style.paddingRight = 6;
            style.marginRight = 4;
            style.marginBottom = 4;
            style.flexGrow = 1;
            style.flexBasis = 0;
            style.minWidth = 80;

            caption = new Label(caption_text.ToUpperInvariant());
            caption.style.color = FidoTheme.Muted;
            caption.style.fontSize = 10;
            caption.style.letterSpacing = 1;
            caption.style.paddingLeft = caption.style.paddingRight = caption.style.paddingTop = caption.style.paddingBottom = 0;
            caption.style.marginLeft = caption.style.marginRight = 0;
            Add(caption);

            value = new Label("-");
            value.style.color = FidoTheme.Fg;
            value.style.fontSize = 15;
            value.style.unityFontStyleAndWeight = FontStyle.Bold;
            value.style.paddingLeft = value.style.paddingRight = value.style.paddingTop = value.style.paddingBottom = 0;
            value.style.marginLeft = value.style.marginRight = 0;
            Add(value);
            parent?.Add(this);
        }

        public void Set(string text, Tone tone = Tone.Normal)
        {
            if (value.text != text) value.text = text;
            value.style.color = FidoKit.ToneColor(tone);
        }
    }

    /// <summary>A status pill: rounded outline in the tone's colour, uppercase bold text.</summary>
    public class Pill : Label
    {
        public Pill(VisualElement parent = null)
        {
            AddToClassList("fido-pill");
            style.fontSize = 11;
            style.unityFontStyleAndWeight = FontStyle.Bold;
            style.letterSpacing = 1;
            style.paddingTop = style.paddingBottom = 1;
            style.paddingLeft = style.paddingRight = 8;
            style.marginLeft = 0;
            style.marginRight = 6;
            style.alignSelf = Align.Center;
            style.borderTopWidth = style.borderBottomWidth = style.borderLeftWidth = style.borderRightWidth = 1;
            style.borderTopLeftRadius = style.borderTopRightRadius = style.borderBottomLeftRadius = style.borderBottomRightRadius = 9;
            parent?.Add(this);
        }

        public void Set(string text, Tone tone)
        {
            text = text.ToUpperInvariant();
            if (this.text != text) this.text = text;
            var c = FidoKit.ToneColor(tone);
            style.color = c;
            style.borderTopColor = style.borderBottomColor = style.borderLeftColor = style.borderRightColor = c;
        }
    }

    /// <summary>An event list: one line per entry, newest first, each with a tone dot.</summary>
    public class EventList : VisualElement
    {
        readonly List<(Label text, VisualElement dot)> rows = new List<(Label, VisualElement)>();

        public EventList(VisualElement parent = null)
        {
            AddToClassList("fido-events");
            parent?.Add(this);
        }

        /// <summary>Shows exactly these entries (reuses rows, so it's cheap to call every UI update).</summary>
        public void Set(IList<(string text, Tone tone)> entries)
        {
            while (rows.Count < entries.Count)
            {
                var row = new VisualElement();
                row.style.flexDirection = FlexDirection.Row;
                row.style.alignItems = Align.Center;
                row.style.borderBottomWidth = 1;
                row.style.borderBottomColor = FidoTheme.Line;
                row.style.paddingTop = row.style.paddingBottom = 1;
                var dot = new VisualElement();
                dot.style.width = dot.style.height = 6;
                dot.style.borderTopLeftRadius = dot.style.borderTopRightRadius = dot.style.borderBottomLeftRadius = dot.style.borderBottomRightRadius = 3;
                dot.style.marginRight = 6;
                dot.style.flexShrink = 0;
                row.Add(dot);
                var l = new Label();
                l.style.color = FidoTheme.Fg;
                l.style.fontSize = 12;
                l.style.whiteSpace = WhiteSpace.Normal;
                l.style.flexShrink = 1;
                l.style.paddingLeft = l.style.paddingRight = l.style.paddingTop = l.style.paddingBottom = 0;
                l.style.marginLeft = l.style.marginRight = 0;
                row.Add(l);
                Add(row);
                rows.Add((l, dot));
            }
            for (int i = 0; i < rows.Count; i++)
            {
                bool on = i < entries.Count;
                rows[i].text.parent.style.display = on ? DisplayStyle.Flex : DisplayStyle.None;
                if (!on) continue;
                if (rows[i].text.text != entries[i].text) rows[i].text.text = entries[i].text;
                rows[i].dot.style.backgroundColor = FidoKit.ToneColor(entries[i].tone);
            }
        }
    }

    /// <summary>
    /// A thin line chart (Painter2D): one or more series on shared axes, faint grid, dashed series for
    /// predictions. The y range covers all points; the x range is the first series' x span.
    /// </summary>
    public class LineChart : VisualElement
    {
        public class Series
        {
            public List<Vector2> points = new List<Vector2>();
            public Color color = FidoTheme.Accent;
            public bool dashed;
        }

        public readonly List<Series> series = new List<Series>();
        public float? y_min, y_max;

        public LineChart(float height = 80, VisualElement parent = null)
        {
            AddToClassList("fido-chart");
            style.height = height;
            style.marginBottom = 4;
            generateVisualContent += Draw;
            parent?.Add(this);
        }

        public Series Add(Color color, bool dashed = false)
        {
            var s = new Series { color = color, dashed = dashed };
            series.Add(s);
            return s;
        }

        public void Changed() => MarkDirtyRepaint();

        void Draw(MeshGenerationContext ctx)
        {
            var r = contentRect;
            if (r.width < 10 || r.height < 10) return;
            var p = ctx.painter2D;

            float x0 = float.MaxValue, x1 = float.MinValue, y0 = float.MaxValue, y1 = float.MinValue;
            foreach (var s in series)
                foreach (var v in s.points)
                {
                    x0 = Mathf.Min(x0, v.x); x1 = Mathf.Max(x1, v.x);
                    y0 = Mathf.Min(y0, v.y); y1 = Mathf.Max(y1, v.y);
                }
            if (y_min.HasValue) y0 = y_min.Value;
            if (y_max.HasValue) y1 = y_max.Value;

            // grid: 4 horizontal lines
            p.lineWidth = 1;
            p.strokeColor = FidoTheme.Grid;
            for (int i = 0; i <= 4; i++)
            {
                float y = r.yMin + r.height * i / 4f;
                p.BeginPath(); p.MoveTo(new Vector2(r.xMin, y)); p.LineTo(new Vector2(r.xMax, y)); p.Stroke();
            }
            if (x1 <= x0 || y1 <= y0) return;

            Vector2 Px(Vector2 v) => new Vector2(
                r.xMin + (v.x - x0) / (x1 - x0) * r.width,
                r.yMax - (Mathf.Clamp(v.y, y0, y1) - y0) / (y1 - y0) * r.height);

            p.lineCap = LineCap.Round;
            p.lineJoin = LineJoin.Round;
            foreach (var s in series)
            {
                if (s.points.Count < 2) continue;
                p.strokeColor = s.color;
                p.lineWidth = 1.5f;
                if (!s.dashed)
                {
                    p.BeginPath();
                    p.MoveTo(Px(s.points[0]));
                    for (int i = 1; i < s.points.Count; i++) p.LineTo(Px(s.points[i]));
                    p.Stroke();
                    continue;
                }
                // dashed: 5 px on, 4 px off along the polyline
                const float on = 5, off = 4;
                float phase = 0;
                for (int i = 1; i < s.points.Count; i++)
                {
                    Vector2 a = Px(s.points[i - 1]), b = Px(s.points[i]);
                    float len = Vector2.Distance(a, b), t = 0;
                    while (t < len)
                    {
                        float cycle = phase % (on + off);
                        float step = Mathf.Min(len - t, cycle < on ? on - cycle : on + off - cycle);
                        if (cycle < on)
                        {
                            p.BeginPath();
                            p.MoveTo(Vector2.Lerp(a, b, t / len));
                            p.LineTo(Vector2.Lerp(a, b, (t + step) / len));
                            p.Stroke();
                        }
                        t += step; phase += step;
                    }
                }
            }
        }
    }
}
