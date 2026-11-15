using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

namespace K2D2.Training
{
    /// <summary>
    /// A small line chart for the Training tab (2026-11-15): one or two series per flight, an
    /// optional reference line (e.g. the debris cap), min/max labels on the y axis and flight
    /// numbers on the x axis. Failed flights (NaN, or below FailBelow) are drawn as red ticks at
    /// the bottom instead of dragging the scale down to the -1000 fail score.
    /// </summary>
    public class TrainingChart : VisualElement
    {
        public struct Series
        {
            public string name;
            public Color color;
            public bool dashed;
            public double[] values;
        }

        public const double FailBelow = -100;

        readonly VisualElement plot;
        readonly Label title_label, legend_label, y_max_label, y_min_label, x_label;
        readonly List<Series> series = new();
        double reference = double.NaN;
        string reference_name;
        double y_lo, y_hi;
        int count;

        static readonly Color Grid = new Color(1, 1, 1, 0.12f);
        static readonly Color RefColor = new Color(0.45f, 0.85f, 0.5f, 0.7f);
        static readonly Color FailColor = new Color(0.95f, 0.35f, 0.3f, 1f);

        public TrainingChart(string title, float height = 110)
        {
            style.marginTop = 8;
            title_label = new Label(title);
            title_label.style.unityFontStyleAndWeight = FontStyle.Bold;
            Add(title_label);

            var row = new VisualElement();
            row.style.flexDirection = FlexDirection.Row;
            Add(row);

            var axis = new VisualElement();
            axis.style.width = 42;
            axis.style.height = height;
            axis.style.justifyContent = Justify.SpaceBetween;
            y_max_label = SmallLabel(axis);
            y_min_label = SmallLabel(axis);
            y_max_label.style.unityTextAlign = TextAnchor.UpperRight;
            y_min_label.style.unityTextAlign = TextAnchor.LowerRight;
            y_max_label.style.paddingRight = y_min_label.style.paddingRight = 4;
            row.Add(axis);

            plot = new VisualElement();
            plot.style.flexGrow = 1;
            plot.style.height = height;
            plot.style.borderLeftWidth = plot.style.borderBottomWidth = 1;
            plot.style.borderLeftColor = plot.style.borderBottomColor = new Color(1, 1, 1, 0.35f);
            plot.generateVisualContent += Draw;
            row.Add(plot);

            x_label = SmallLabel(this);
            x_label.style.marginLeft = 42;
            legend_label = SmallLabel(this);
            legend_label.enableRichText = true;
        }

        static Label SmallLabel(VisualElement parent)
        {
            var l = new Label();
            l.AddToClassList("k2-slider-value-label");
            l.style.whiteSpace = WhiteSpace.Normal;
            parent.Add(l);
            return l;
        }

        /// <summary>Replaces the data. unit goes on the axis labels ("km", "m/s", "").</summary>
        public void SetData(IList<Series> data, string unit, double referenceValue = double.NaN, string referenceName = null)
        {
            series.Clear();
            series.AddRange(data);
            reference = referenceValue;
            reference_name = referenceName;

            count = 0;
            double lo = double.PositiveInfinity, hi = double.NegativeInfinity;
            foreach (var s in series)
            {
                count = Math.Max(count, s.values.Length);
                foreach (var v in s.values)
                    if (Good(v)) { lo = Math.Min(lo, v); hi = Math.Max(hi, v); }
            }
            if (!double.IsNaN(reference)) { lo = Math.Min(lo, reference); hi = Math.Max(hi, reference); }
            if (double.IsInfinity(lo)) { lo = 0; hi = 1; }
            double pad = Math.Max(1e-6, (hi - lo) * 0.08);
            if (hi - lo < 1e-6) pad = Math.Max(1, Math.Abs(hi) * 0.1);
            y_lo = lo - pad;
            y_hi = hi + pad;

            y_max_label.text = $"{y_hi:0.#}{unit}";
            y_min_label.text = $"{y_lo:0.#}{unit}";
            x_label.text = count == 0 ? "no flights yet" : count == 1 ? "flight 1" : $"flight 1 → {count}";
            var legend = "";
            foreach (var s in series)
                legend += $"<color=#{ColorUtility.ToHtmlStringRGB(s.color)}>{(s.dashed ? "- - -" : "-----")}</color> {s.name}   ";
            if (!double.IsNaN(reference) && reference_name != null)
                legend += $"<color=#{ColorUtility.ToHtmlStringRGB(RefColor)}>. . .</color> {reference_name}";
            legend_label.text = legend.TrimEnd();
            plot.MarkDirtyRepaint();
        }

        static bool Good(double v) => !double.IsNaN(v) && !double.IsInfinity(v) && v > FailBelow;

        Vector2 Point(int i, double v, Rect r)
        {
            float x = count <= 1 ? r.width / 2 : (float)(i * (r.width - 12) / (count - 1)) + 6;
            float y = (float)((y_hi - v) / (y_hi - y_lo) * r.height);
            return new Vector2(x, Mathf.Clamp(y, 0, r.height));
        }

        void Draw(MeshGenerationContext ctx)
        {
            var r = plot.contentRect;
            if (r.width < 10 || r.height < 10) return;
            var p = ctx.painter2D;

            // light grid: quarter lines
            p.strokeColor = Grid;
            p.lineWidth = 1;
            for (int g = 1; g < 4; g++)
            {
                float y = r.height * g / 4f;
                p.BeginPath(); p.MoveTo(new Vector2(0, y)); p.LineTo(new Vector2(r.width, y)); p.Stroke();
            }

            if (!double.IsNaN(reference))
            {
                p.strokeColor = RefColor;
                float y = Point(0, reference, r).y;
                for (float x = 0; x < r.width; x += 8)
                {
                    p.BeginPath(); p.MoveTo(new Vector2(x, y)); p.LineTo(new Vector2(Math.Min(r.width, x + 3), y)); p.Stroke();
                }
            }

            foreach (var s in series)
            {
                p.strokeColor = s.color;
                p.fillColor = s.color;
                p.lineWidth = 2;
                // line through the good points, broken at failures
                bool open = false;
                Vector2 prev = default;
                for (int i = 0; i < s.values.Length; i++)
                {
                    if (!Good(s.values[i])) { open = false; continue; }
                    var pt = Point(i, s.values[i], r);
                    if (open)
                    {
                        if (s.dashed) DashedLine(p, prev, pt);
                        else { p.BeginPath(); p.MoveTo(prev); p.LineTo(pt); p.Stroke(); }
                    }
                    prev = pt;
                    open = true;
                }
                // markers
                for (int i = 0; i < s.values.Length; i++)
                {
                    double v = s.values[i];
                    if (Good(v))
                    {
                        var pt = Point(i, v, r);
                        p.BeginPath(); p.Arc(pt, s.dashed ? 2f : 3f, 0, 360); p.Fill();
                    }
                    else if (!s.dashed && !double.IsNaN(v))
                    {
                        // a failed flight (fail score): red tick on the x axis
                        var pt = Point(i, y_lo, r);
                        p.strokeColor = FailColor;
                        p.BeginPath(); p.MoveTo(new Vector2(pt.x, r.height)); p.LineTo(new Vector2(pt.x, r.height - 10)); p.Stroke();
                        p.strokeColor = s.color;
                    }
                }
            }
        }

        static void DashedLine(Painter2D p, Vector2 a, Vector2 b)
        {
            float len = Vector2.Distance(a, b);
            if (len < 0.5f) return;
            var dir = (b - a) / len;
            for (float d = 0; d < len; d += 7)
            {
                p.BeginPath(); p.MoveTo(a + dir * d); p.LineTo(a + dir * Math.Min(len, d + 4)); p.Stroke();
            }
        }
    }
}
