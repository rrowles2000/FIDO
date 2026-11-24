using Unity.Properties;
using UnityEngine.UIElements;

namespace K2UI.Tabs
{

    /// <summary>
    /// a simple visual element just used to contains label and icon
    /// </summary>
    // UxmlFactory/UxmlTraits -> [UxmlElement]/[UxmlAttribute] (see Group.cs's class comment for
    // why). "name" itself needs no special handling any more - UI Toolkit's own attribute
    // application always sets it now, regardless of custom control type - so
    // TabbedPage.ShowContent()'s page.Show(page.name == code), K2Page.Init()'s
    // panels.Q<TabPage>(code), and setButton() below all keep working unchanged. TabPage is never
    // actually instantiated from a UXML tag in this project though (K2D2_Window.uxml's tab content
    // is <ui:Instance> template references, not literal <K2UI.Tabs.TabPage> tags) - only ever via
    // `new TabPage()` - so none of this was actually exercised via Init() before either way.
    [UxmlElement]
    public partial class TabPage : VisualElement
    {
        public string _label;

        [CreateProperty]
        [UxmlAttribute("label")]
        public string label
        {
            get { return _label; }
            set { 
                    if (value == _label) return;
                    _label = value; 
                    
                    if (tab_button != null)
                    {
                        tab_button.label = label;
                    }
                }
        }

        public TabButton tab_button;
        public void setButton(TabButton bt)
        {
            bt.name = name;   // first: the icon is chosen by the tab's code name
            bt.label = label;
        }
    }


    /// <summary>
    /// TabButton have two states : active (showing current content) and lighted (pilot is on)
    /// </summary>
    // UxmlFactory/UxmlTraits -> [UxmlElement]/[UxmlAttribute] (see Group.cs's class comment for
    // why). TabButton, like TabsBar and TabPage, is never actually instantiated from a UXML tag in
    // this project - only ever via `new TabButton()` from TabbedPage/TabPage's own code - so its
    // old Init() (and the "Tab Button"/false/false defaults it unconditionally applied) never
    // actually ran in practice. Converted for consistency/future-proofing anyway.
    [UxmlElement]
    public partial class TabButton : VisualElement
    {
        // Must expose your element class to a { get; set; } property that has the same name
        // as the name you set in your UXML attribute description with the camel case format
        public string _label;

        [CreateProperty]
        [UxmlAttribute("label")]
        public string label
        {
            get { return _label; }
            set
            {
                _label = value;
                el_label.text = value;
                icon.Set(name, value);
            }
        }
        bool _active;

        [CreateProperty]
        [UxmlAttribute("active")]
        public bool Active
        {
            get { return _active; }
            set
            {
                if (_active == value)
                    return;

                var evt = ChangeEvent<bool>.GetPooled(_active, value);
                evt.target = this;
                _active = value;

                EnableInClassList(activeUss, _active);
                ApplyLook();
                SendEvent(evt);
            }
        }

        bool _lighted;

        [CreateProperty]
        [UxmlAttribute("lighted")]
        public bool Lighted
        {
            get { return _lighted; }
            set
            {
                _lighted = value;
                el_light.EnableInClassList(lightedUss, _lighted);
                el_light.style.display = _lighted ? DisplayStyle.Flex : DisplayStyle.None;
            }
        }

        // In the spirit of the BEM standard, the TabButton has its own block class and two element classes. It also
        // has a class that represents the enabled state of the toggle.
        public static readonly string ussClassName = "k2-tab-button";
        public static readonly string activeUss = ussClassName+"--active";

        public static readonly string usslightName = "tab_light";

        public static readonly string lightedUss = usslightName+"--lighted";

        Label el_label;
        VisualElement el_light;
        readonly TabIcon icon = new TabIcon();

        static readonly UnityEngine.Color IconIdle = new UnityEngine.Color(0.72f, 0.76f, 0.8f);
        static readonly UnityEngine.Color IconActive = new UnityEngine.Color(1f, 1f, 1f);
        static readonly UnityEngine.Color FillActive = new UnityEngine.Color(0.2f, 0.42f, 0.8f, 0.85f);
        static readonly UnityEngine.Color EdgeActive = new UnityEngine.Color(0.45f, 0.7f, 1f);
        static readonly UnityEngine.Color EdgeIdle = new UnityEngine.Color(1f, 1f, 1f, 0.12f);
        static readonly UnityEngine.Color FillIdle = new UnityEngine.Color(0f, 0f, 0f, 0f);

        // 2026-11-15: square icon buttons, the name as a hover tooltip (was a text label). Every
        // state colour is set inline, so the bundle's pill-tab styles (active background, the
        // stretched "pilot running" light bar) no longer leave a second tab looking selected.
        void ApplyLook()
        {
            icon.SetColor(_active ? IconActive : IconIdle);
            var edge = _active ? EdgeActive : EdgeIdle;
            style.backgroundColor = _active ? FillActive : FillIdle;
            style.borderTopColor = style.borderBottomColor = style.borderLeftColor = style.borderRightColor = edge;
            style.borderTopWidth = style.borderBottomWidth = style.borderLeftWidth = style.borderRightWidth = 1;
        }
        
        // This constructor allows users to set the contents of the label.
        public TabButton()
        {
            el_light = new VisualElement();
            el_light.name = "tab_light";
            el_light.AddToClassList(usslightName);
            // a small green dot in the corner while the tab's pilot runs (the bundle's light is a
            // bar sized for the old wide tabs)
            el_light.style.position = Position.Absolute;
            el_light.style.top = 2;
            el_light.style.right = 2;
            el_light.style.left = StyleKeyword.Auto;
            el_light.style.width = el_light.style.height = 6;
            el_light.style.borderTopLeftRadius = el_light.style.borderTopRightRadius = el_light.style.borderBottomLeftRadius = el_light.style.borderBottomRightRadius = 3;
            el_light.style.backgroundImage = StyleKeyword.None;
            el_light.style.backgroundColor = new UnityEngine.Color(0.3f, 0.85f, 0.35f);
            el_light.style.display = DisplayStyle.None;
            el_light.pickingMode = PickingMode.Ignore;
            Add(el_light);

            el_label = new Label();
            el_label.style.display = DisplayStyle.None; // the name is the tooltip now
            Add(el_label);
            Add(icon);

            // overrides the bundle's pill-shaped k2-tab-button sizing. Every tab splits the bar equally:
            // K2D2.uss gives only the original tabs (#node, #lift, ... #about) flex-grow: 1, so the
            // code-built ones (Mission, Sats, Training) stayed 34 px while the rest stretched.
            // Inline styles win over the bundle's USS.
            style.height = 34;
            style.width = StyleKeyword.Auto;
            style.flexBasis = 0;
            style.flexGrow = 1;
            style.flexShrink = 1;
            style.minWidth = 34;
            style.marginRight = 0;
            style.paddingLeft = style.paddingRight = style.paddingTop = style.paddingBottom = 0;
            style.borderTopLeftRadius = style.borderTopRightRadius = style.borderBottomLeftRadius = style.borderBottomRightRadius = 6;
            style.justifyContent = Justify.Center;
            style.alignItems = Align.Center;
            style.marginLeft = 3;
            HoverTip.Attach(this, () => _label);
            ApplyLook();

            // Style the control overall.
            AddToClassList(ussClassName);
            this.AddManipulator(new Clickable(evt => {
                Active = true;
                }));
        }

    }
}