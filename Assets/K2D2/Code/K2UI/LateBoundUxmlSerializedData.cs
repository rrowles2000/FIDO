using System;
using System.Reflection;
using UnityEngine;
using UnityEngine.UIElements;

namespace K2UI
{
    // Redux stable 0.2.8.5 shim. The 1.3.0 UI was authored against the beta SDK, whose
    // uitkforksp2.controls.Runtime defines UitkForKsp2.Controls.LateBoundUxmlSerializedData: a
    // wrapper that stores each K2D2 custom control's UxmlSerializedData as JSON plus its type name.
    // Stable's controls assembly has no such type, so the bundle's managed references are repointed
    // at this class (asm K2D2, ns K2UI) and it unwraps the payload itself.
    [Serializable]
    public class LateBoundUxmlSerializedData : UxmlSerializedData, ISerializationCallbackReceiver
    {
        [SerializeField] private string _serializedDataTypeName;
        [SerializeField] private string _serializedDataJson;

        [NonSerialized] private UxmlSerializedData _inner;

        private static readonly FieldInfo UxmlAssetIdField =
            typeof(UxmlSerializedData).GetField("uxmlAssetId", BindingFlags.Instance | BindingFlags.NonPublic);

        private UxmlSerializedData Inner
        {
            get
            {
                if (_inner != null) return _inner;
                var type = Type.GetType(_serializedDataTypeName);
                if (type == null)
                {
                    Debug.LogError($"[K2D2] LateBoundUxmlSerializedData: type '{_serializedDataTypeName}' not found");
                    return null;
                }
                _inner = (UxmlSerializedData)JsonUtility.FromJson(_serializedDataJson, type);
                if (UxmlAssetIdField != null && _inner != null)
                    UxmlAssetIdField.SetValue(this, UxmlAssetIdField.GetValue(_inner));
                return _inner;
            }
        }

        public override object CreateInstance() => Inner?.CreateInstance();

        public override void Deserialize(object obj) => Inner?.Deserialize(obj);

        public void OnBeforeSerialize() { }

        public void OnAfterDeserialize()
        {
            _inner = null;
            try { _ = Inner; } catch (Exception) { /* resolved lazily on first use instead */ }
        }
    }
}
