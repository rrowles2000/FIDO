using System;
using System.Reflection;
using System.Runtime.CompilerServices;
using HarmonyLib;
using UnityEngine;
using UnityEngine.UIElements;

namespace K2UI
{
    // Redux stable 0.2.8.5 replacement for the beta's UitkForKsp2.Controls.LateBoundUxmlSerializedData.
    //
    // The 1.3.0 UXML stores each FIDO custom control's UxmlSerializedData as JSON + type name inside
    // that beta-only wrapper. Unity can only resolve [SerializeReference] types from assemblies it
    // knows at startup, so a wrapper defined in Fido.dll is "Unknown managed type". Instead the
    // bundle is rewritten (ksp2-tools/uss-remap/retarget_carrier.py) so each wrapper becomes a plain
    // VisualElement.UxmlSerializedData carrying:
    //   viewDataKey          = Marker
    //   tooltip              = the real UxmlSerializedData type name
    //   dataSourcePathString = the real UxmlSerializedData as JSON
    // and these Harmony prefixes divert CreateInstance/Deserialize for such carriers to the real type.
    internal static class LateBoundCarrier
    {
        // bundle data written by retarget_carrier.py: keep the old marker, the bundle is not rebuilt
        public const string Marker = "K2D2.LateBound";

        private static readonly FieldInfo ViewDataKey = Field("viewDataKey");
        private static readonly FieldInfo Tooltip = Field("tooltip");
        private static readonly FieldInfo Json = Field("dataSourcePathString");

        private static readonly ConditionalWeakTable<UxmlSerializedData, UxmlSerializedData> Inners = new();

        private static FieldInfo Field(string name) =>
            typeof(VisualElement.UxmlSerializedData).GetField(name, BindingFlags.Instance | BindingFlags.NonPublic);

        public static void Apply(Harmony harmony)
        {
            var t = typeof(VisualElement.UxmlSerializedData);
            harmony.Patch(AccessTools.Method(t, nameof(UxmlSerializedData.CreateInstance)),
                prefix: new HarmonyMethod(typeof(LateBoundCarrier), nameof(CreateInstancePrefix)));
            harmony.Patch(AccessTools.Method(t, nameof(UxmlSerializedData.Deserialize), new[] { typeof(object) }),
                prefix: new HarmonyMethod(typeof(LateBoundCarrier), nameof(DeserializePrefix)));
        }

        private static UxmlSerializedData Inner(UxmlSerializedData carrier)
        {
            if (carrier.GetType() != typeof(VisualElement.UxmlSerializedData)) return null;
            if (!(ViewDataKey?.GetValue(carrier) is string key) || key != Marker) return null;
            if (Inners.TryGetValue(carrier, out var cached)) return cached;

            var typeName = (string)Tooltip.GetValue(carrier);
            // The prebuilt bundle names the types with the old assembly ("K2UI.K2Compass+..., K2D2").
            // The assembly is now Fido, so look the bare type name up in this assembly.
            var comma = typeName.IndexOf(',');
            var bareName = comma < 0 ? typeName : typeName.Substring(0, comma).Trim();
            var type = typeof(LateBoundCarrier).Assembly.GetType(bareName) ?? Type.GetType(typeName);
            if (type == null)
            {
                Debug.LogError($"[FIDO] LateBoundCarrier: type '{typeName}' not found");
                return null;
            }
            var inner = (UxmlSerializedData)JsonUtility.FromJson((string)Json.GetValue(carrier), type);
            Inners.Add(carrier, inner);
            return inner;
        }

        private static bool CreateInstancePrefix(UxmlSerializedData __instance, ref object __result)
        {
            var inner = Inner(__instance);
            if (inner == null) return true;
            __result = inner.CreateInstance();
            return false;
        }

        private static bool DeserializePrefix(UxmlSerializedData __instance, object obj)
        {
            var inner = Inner(__instance);
            if (inner == null) return true;
            inner.Deserialize(obj);
            return false;
        }
    }
}
