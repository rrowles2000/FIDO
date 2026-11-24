using KSP.Audio;
using KSP.Game;

namespace Fido.KSPService
{
    /// <summary>
    /// Silences the Kerbals' voices (radio chatter "KerbalComm" and cabin reactions "Kerbal IVA") while any
    /// FIDO pilot is running, and gives back the player's own volumes when the last one stops.
    ///
    /// KSPAudioEventManager.SetKerbalCommVolume / SetKerbalIVAVolume only set the live Wwise RTPC (and the
    /// Redux Unity-audio mixer); the saved values stay in PersistentProfileManager, so nothing on disk changes
    /// and a crash mid-burn leaves the player's settings intact. If the player applies audio settings while
    /// muted, the next frame mutes again.
    /// </summary>
    static class KerbalVoiceMute
    {
        static bool muted;

        public static void Update(bool any_pilot_running)
        {
            try
            {
                if (any_pilot_running)
                {
                    if (!muted || KSPAudioEventManager.GetKerbalCommVolume() > 0 || KSPAudioEventManager.GetKerbalIVAVolume() > 0)
                    {
                        if (!muted) FidoPlugin.logger.LogInfo("[FIDO.Audio] Kerbal voices muted (pilot running)");
                        KSPAudioEventManager.SetKerbalCommVolume(0f);
                        KSPAudioEventManager.SetKerbalIVAVolume(0f);
                        muted = true;
                    }
                }
                else if (muted)
                {
                    Restore();
                }
            }
            catch (System.Exception e)
            {
                // audio manager not up yet (scene load): try again next frame
                if (muted) FidoPlugin.logger.LogWarning("[FIDO.Audio] " + e.Message);
            }
        }

        public static void Restore()
        {
            if (!muted) return;
            KSPAudioEventManager.SetKerbalCommVolume(PersistentProfileManager.KerbalCommVolume);
            KSPAudioEventManager.SetKerbalIVAVolume(PersistentProfileManager.KerbalIVAVolume);
            muted = false;
            FidoPlugin.logger.LogInfo("[FIDO.Audio] Kerbal voices restored");
        }
    }
}
