using System.Collections.Generic;
using System.Linq;
using Fido.Controller;

namespace Fido
{
    class FidoPilotsMgr
    {
        static FidoPilotsMgr _instance = null;
        static public FidoPilotsMgr Instance
        {
            get { return _instance; }
        }

        public FidoPilotsMgr()
        {
            _instance = this;
        }

        Dictionary<string, BaseController> pilots = new Dictionary<string, BaseController>();

        public bool isPilotEnabled(string pilotName)
        {
            if (!pilots.ContainsKey(pilotName)) { return false; }

            BaseController pilot = pilots[pilotName];
            return pilot.Enabled;
        }

        public void EnablePilot(string pilotName, bool enabled)
        {
            if (!pilots.ContainsKey(pilotName)) { return; }

            BaseController pilot = pilots[pilotName];
            pilot.Enabled = enabled;
        }

        public void EnableAllPilots(bool enabled)
        {
            foreach(var pilot in pilots.Values)
                pilot.Enabled = enabled;
        }

        public List<string> GetPilotsNames()
        {
            return pilots.Keys.ToList();
        }

        internal void RegisterPilot(string pilotName, BaseController controller)
        {
            if (pilots.ContainsKey(pilotName)) 
            { 
                FidoPlugin.logger.LogWarning($"Registering another pilot named {pilotName}");
                return;
            }

            pilots.Add(pilotName, controller);
        }

    }
}