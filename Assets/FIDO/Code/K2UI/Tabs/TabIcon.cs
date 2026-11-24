using UnityEngine;
using UnityEngine.UIElements;

namespace K2UI.Tabs
{
    /// <summary>
    /// The tab bar's icons (2026-11-15): one line-drawn glyph per tab, on a 22x22 grid, drawn with
    /// Painter2D in the button's colour - so they need no image assets and stay sharp at any UI
    /// scale. The tab's name is its hover tooltip (TabButton). An unknown tab gets its initial.
    /// </summary>
    public class TabIcon : VisualElement
    {
        public string code;
        public Color color = new Color(0.82f, 0.85f, 0.88f);
        readonly Label fallback;

        public TabIcon()
        {
            pickingMode = PickingMode.Ignore;
            style.width = style.height = 22;
            style.alignSelf = Align.Center;
            generateVisualContent += Draw;
            fallback = new Label { pickingMode = PickingMode.Ignore };
            fallback.style.unityTextAlign = TextAnchor.MiddleCenter;
            fallback.style.fontSize = 15;
            fallback.style.unityFontStyleAndWeight = FontStyle.Bold;
            fallback.style.width = fallback.style.height = 22;
            fallback.style.paddingLeft = fallback.style.paddingRight = fallback.style.paddingTop = fallback.style.paddingBottom = 0;
            fallback.style.marginLeft = fallback.style.marginRight = fallback.style.marginTop = fallback.style.marginBottom = 0;
            Add(fallback);
        }

        public void Set(string tabCode, string label)
        {
            code = (tabCode ?? "").ToLowerInvariant();
            bool known = Known(code);
            fallback.style.display = known ? DisplayStyle.None : DisplayStyle.Flex;
            fallback.text = string.IsNullOrEmpty(label) ? "?" : label.Substring(0, 1).ToUpperInvariant();
            MarkDirtyRepaint();
        }

        public void SetColor(Color c)
        {
            color = c;
            fallback.style.color = c;
            MarkDirtyRepaint();
        }

        static bool Known(string c) => c switch
        {
            "mission" or "node" or "lift" or "landing" or "dock" or "attitude" or "sats" or "constellation" or "training" or "about" => true,
            _ => false,
        };

        void Draw(MeshGenerationContext ctx)
        {
            if (!Known(code)) return;
            var r = contentRect;
            if (r.width < 4) return;
            float s = Mathf.Min(r.width, r.height) / 22f;
            Vector2 o = new Vector2((r.width - 22 * s) / 2, (r.height - 22 * s) / 2);
            var p = ctx.painter2D;
            p.strokeColor = color;
            p.fillColor = color;
            p.lineWidth = 1.6f * s;
            p.lineCap = LineCap.Round;
            p.lineJoin = LineJoin.Round;
            Vector2 P(float x, float y) => o + new Vector2(x, y) * s;

            void Line(params Vector2[] pts)
            {
                p.BeginPath();
                p.MoveTo(pts[0]);
                for (int i = 1; i < pts.Length; i++) p.LineTo(pts[i]);
                p.Stroke();
            }
            void Closed(bool fill, params Vector2[] pts)
            {
                p.BeginPath();
                p.MoveTo(pts[0]);
                for (int i = 1; i < pts.Length; i++) p.LineTo(pts[i]);
                p.ClosePath();
                if (fill) p.Fill(); else p.Stroke();
            }
            void Circle(float cx, float cy, float rad, bool fill = false)
            {
                p.BeginPath();
                p.Arc(P(cx, cy), rad * s, 0, 360);
                if (fill) p.Fill(); else p.Stroke();
            }
            void ArcPath(float cx, float cy, float rad, float a0, float a1)
            {
                p.BeginPath();
                p.Arc(P(cx, cy), rad * s, a0, a1);
                p.Stroke();
            }

            switch (code)
            {
                case "mission": // a flag on a pole, with a route line to it
                    Line(P(6, 20), P(6, 3));
                    Closed(false, P(6, 4), P(17, 6.5f), P(6, 10));
                    Line(P(2, 20), P(10, 20));
                    break;

                case "node": // an orbit with the maneuver node marker and its burn vector
                    p.BeginPath();
                    p.Arc(P(11, 12), 8 * s, 200, 520);
                    p.Stroke();
                    Closed(true, P(11, 1.5f), P(14, 6), P(8, 6));
                    Line(P(11, 6), P(11, 9.5f));
                    break;

                case "lift": // a rocket climbing, with exhaust
                    Closed(false, P(11, 2), P(14.5f, 8), P(14.5f, 15), P(7.5f, 15), P(7.5f, 8));
                    Closed(false, P(7.5f, 11), P(4.5f, 16), P(7.5f, 15));
                    Closed(false, P(14.5f, 11), P(17.5f, 16), P(14.5f, 15));
                    Circle(11, 9, 1.4f);
                    Line(P(9.5f, 17.5f), P(9, 20.5f));
                    Line(P(12.5f, 17.5f), P(13, 20.5f));
                    break;

                case "landing": // a lander on its legs over the ground, arrow down
                    Closed(false, P(7.5f, 7), P(14.5f, 7), P(16, 12), P(6, 12));
                    Line(P(7, 12), P(4, 17));
                    Line(P(15, 12), P(18, 17));
                    Line(P(2, 19), P(20, 19));
                    Line(P(11, 1.5f), P(11, 4.5f));
                    Line(P(9.5f, 3.5f), P(11, 5), P(12.5f, 3.5f));
                    break;

                case "dock": // two docking ports closing on each other
                    Line(P(2, 7), P(8, 7), P(8, 15), P(2, 15));
                    Line(P(20, 7), P(14, 7), P(14, 15), P(20, 15));
                    Line(P(9.5f, 11), P(12.5f, 11));
                    Line(P(5, 9.5f), P(5, 12.5f));
                    Line(P(17, 9.5f), P(17, 12.5f));
                    break;

                case "attitude": // a navball: circle, horizon and the vessel chevron
                    Circle(11, 11, 8.5f);
                    Line(P(3, 12.5f), P(19, 9.5f));
                    Line(P(7, 13), P(11, 10), P(15, 13));
                    Circle(11, 11, 0.9f, true);
                    break;

                case "sats":
                case "constellation": // a satellite: body, two solar panels, antenna
                    Closed(false, P(9, 8.5f), P(13, 8.5f), P(13, 13.5f), P(9, 13.5f));
                    Closed(false, P(1.5f, 9), P(7, 9), P(7, 13), P(1.5f, 13));
                    Closed(false, P(15, 9), P(20.5f, 9), P(20.5f, 13), P(15, 13));
                    Line(P(7, 11), P(9, 11));
                    Line(P(13, 11), P(15, 11));
                    Line(P(11, 8.5f), P(11, 5));
                    ArcPath(11, 5, 2.5f, 200, 340);
                    Line(P(11, 13.5f), P(11, 17));
                    break;

                case "training": // a rising line chart with a repeat arrow
                    Line(P(2.5f, 19.5f), P(2.5f, 3), P(2.5f, 19.5f), P(19.5f, 19.5f));
                    Line(P(5, 16), P(9, 12), P(12, 14), P(18, 6));
                    Line(P(15, 6), P(18, 6), P(18, 9));
                    break;

                case "about": // "i" in a circle
                    Circle(11, 11, 8.5f);
                    Circle(11, 6.8f, 1.1f, true);
                    Line(P(11, 10), P(11, 16));
                    break;
            }
        }
    }
}
