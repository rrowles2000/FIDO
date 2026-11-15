using System;
using UnityEngine;
using UnityEngine.UIElements;

namespace K2UI
{
    /// <summary>
    /// Hover tooltips for the runtime UI (2026-11-15). UI Toolkit's own `tooltip` property is only
    /// shown in the Editor, so at runtime nothing ever appeared. Attach() shows a floating label at
    /// the pointer while it is over an element.
    ///
    /// StartSweeping() also turns the window's yellow text (the k2-slider-value-label class: hints,
    /// notes, check results) into a small "i" badge that shows the text on hover, so the tabs stay
    /// compact. Slider value read-outs and the heading read-out stay visible - they are values, not
    /// text. The sweep runs every half second, so labels built later (the Mission editor rebuilds
    /// its lists) are converted too, and a badge hides while its text is empty.
    /// </summary>
    public static class HoverTip
    {
        public const string HintClass = "k2-slider-value-label";
        const string ConvertedClass = "k2-hint--tooltip";
        const string BadgeClass = "k2-hint-badge";

        static Label tip;
        static readonly System.Collections.Generic.HashSet<VisualElement> swept = new();

        public static void Attach(VisualElement target, Func<string> text)
        {
            target.RegisterCallback<PointerEnterEvent>(e => Show(target, text?.Invoke(), e.position));
            target.RegisterCallback<PointerMoveEvent>(e => Move(e.position));
            target.RegisterCallback<PointerLeaveEvent>(e => Hide());
            target.RegisterCallback<DetachFromPanelEvent>(e => Hide());
        }

        public static void Attach(VisualElement target, string text) => Attach(target, () => text);

        // the tooltip lives in the window's own root (topmost element below the panel), so it gets
        // the window's fonts and stylesheets
        static VisualElement RootOf(VisualElement el)
        {
            var root = el;
            while (root.parent != null && root.parent != el.panel?.visualTree) root = root.parent;
            return root;
        }

        static void Show(VisualElement target, string text, Vector2 panelPos)
        {
            if (string.IsNullOrWhiteSpace(text) || target.panel == null) { Hide(); return; }
            var root = RootOf(target);
            if (tip == null)
            {
                tip = new Label { pickingMode = PickingMode.Ignore };
                tip.style.position = Position.Absolute;
                tip.style.maxWidth = 300;
                tip.style.whiteSpace = WhiteSpace.Normal;
                tip.style.fontSize = 13;
                tip.style.color = new Color(0.93f, 0.95f, 0.97f);
                tip.style.backgroundColor = new Color(0.07f, 0.09f, 0.12f, 0.96f);
                tip.style.borderTopWidth = tip.style.borderBottomWidth = tip.style.borderLeftWidth = tip.style.borderRightWidth = 1;
                var edge = new Color(0.45f, 0.52f, 0.6f, 0.8f);
                tip.style.borderTopColor = tip.style.borderBottomColor = tip.style.borderLeftColor = tip.style.borderRightColor = edge;
                tip.style.borderTopLeftRadius = tip.style.borderTopRightRadius = tip.style.borderBottomLeftRadius = tip.style.borderBottomRightRadius = 4;
                tip.style.paddingTop = tip.style.paddingBottom = 5;
                tip.style.paddingLeft = tip.style.paddingRight = 8;
            }
            if (tip.parent != root) root.Add(tip);
            tip.text = text;
            tip.style.display = DisplayStyle.Flex;
            tip.BringToFront();
            Move(panelPos);
        }

        static void Move(Vector2 panelPos)
        {
            if (tip?.parent == null || tip.style.display == DisplayStyle.None) return;
            var root = tip.parent;
            Vector2 p = root.WorldToLocal(panelPos);
            float w = float.IsNaN(tip.layout.width) ? 200 : tip.layout.width;
            float h = float.IsNaN(tip.layout.height) ? 30 : tip.layout.height;
            float rw = root.layout.width, rh = root.layout.height;
            float x = p.x + 14, y = p.y + 18;
            if (!float.IsNaN(rw) && rw > 0 && x + w > rw) x = Mathf.Max(0, p.x - w - 6);
            if (!float.IsNaN(rh) && rh > 0 && y + h > rh) y = Mathf.Max(0, p.y - h - 8);
            tip.style.left = x;
            tip.style.top = y;
        }

        public static void Hide()
        {
            if (tip != null) tip.style.display = DisplayStyle.None;
        }

        // ---------------------------------------------------------------- yellow text -> badge

        public static void StartSweeping(VisualElement root)
        {
            if (root == null || !swept.Add(root)) return;
            root.schedule.Execute(() => Sweep(root)).Every(500);
            Sweep(root);
        }

        static bool IsValueReadout(VisualElement l)
        {
            if (l.name == "heading_label") return true;
            for (var p = l.parent; p != null; p = p.parent)
                if (p is K2Slider || p is K2SliderInt) return true;
            return false;
        }

        public static void Sweep(VisualElement root)
        {
            root.Query<Label>(className: HintClass).ForEach(l =>
            {
                if (l.ClassListContains(ConvertedClass) || l.ClassListContains(BadgeClass) || IsValueReadout(l) || l.parent == null) return;
                l.AddToClassList(ConvertedClass);
                var badge = MakeBadge(() => l.text);
                badge.userData = l;
                l.parent.Insert(l.parent.IndexOf(l), badge);
                l.style.display = DisplayStyle.None;
            });
            // keep badges in step with their text
            root.Query<Label>(className: BadgeClass).ForEach(b =>
            {
                var src = b.userData as Label;
                bool show = src != null && !string.IsNullOrWhiteSpace(src.text);
                b.style.display = show ? DisplayStyle.Flex : DisplayStyle.None;
            });
        }

        /// <summary>A small round "i" that shows text on hover.</summary>
        public static Label MakeBadge(Func<string> text)
        {
            var b = new Label("i");
            b.AddToClassList(BadgeClass);
            b.style.width = b.style.height = 17;
            b.style.minWidth = 17;
            b.style.flexShrink = 0;
            b.style.alignSelf = Align.FlexStart;
            b.style.unityTextAlign = TextAnchor.MiddleCenter;
            b.style.unityFontStyleAndWeight = FontStyle.Bold;
            b.style.fontSize = 12;
            b.style.marginTop = b.style.marginBottom = 2;
            b.style.marginLeft = b.style.marginRight = 2;
            b.style.paddingTop = b.style.paddingBottom = b.style.paddingLeft = b.style.paddingRight = 0;
            var c = new Color(0.95f, 0.79f, 0.3f); // the same amber the text had
            b.style.color = c;
            b.style.borderTopWidth = b.style.borderBottomWidth = b.style.borderLeftWidth = b.style.borderRightWidth = 1;
            b.style.borderTopColor = b.style.borderBottomColor = b.style.borderLeftColor = b.style.borderRightColor = c;
            b.style.borderTopLeftRadius = b.style.borderTopRightRadius = b.style.borderBottomLeftRadius = b.style.borderBottomRightRadius = 9;
            Attach(b, text);
            return b;
        }
    }
}
