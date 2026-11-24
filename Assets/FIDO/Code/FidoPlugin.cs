using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using JetBrains.Annotations;
using SpaceWarp2.UI.API.Appbar;
using Fido.UI;
using UitkForKsp2.API;
using UnityEngine;
using KTools;
using Fido.KSPService;
using KSP.Game;
using KSP.Messages;
using Fido.Controller;

using Fido.Lift;
using Fido.Landing;
using Fido.Node;
using Redux.ExtraModTypes;
using UnityEngine.ResourceManagement.AsyncOperations;
using ILogger = ReduxLib.Logging.ILogger;

namespace Fido
{
    internal class L
    {
        public static void Log(string txt)
        {
            FidoPlugin.logger.LogInfo(txt);
        }

        public static void Vector3(string label, Vector3 value)
        {
            FidoPlugin.logger.LogInfo(label + " : " + StrTool.Vector3ToString(value));
        }
    }

    public class FidoPlugin : KerbalMod
    {


        private static string _assemblyFolder;
        private static string AssemblyFolder =>
            _assemblyFolder ??= Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location);

        private static string _settingsPath;
        private static string SettingsPath =>
            _settingsPath ??= Path.Combine(AssemblyFolder, "fido_settings.json");

        // Landing's Atmo/Vacuum profile split (see LandingSettings.cs/KTools.SettingsFile.cs) -
        // two genuinely separate physical files, so tuning one profile can never touch the other's
        // saved values. Same AssemblyFolder-relative pattern as SettingsPath above.
        private static string _atmoLandingSettingsPath;
        private static string AtmoLandingSettingsPath =>
            _atmoLandingSettingsPath ??= Path.Combine(AssemblyFolder, "fido_landing_atmo.json");

        private static string _vacLandingSettingsPath;
        private static string VacLandingSettingsPath =>
            _vacLandingSettingsPath ??= Path.Combine(AssemblyFolder, "fido_landing_vac.json");


        /// Singleton instance of the plugin class
        [PublicAPI] public static FidoPlugin Instance { get; set; }

        // AppBar button IDs
        internal const string ToolbarFlightButtonID = "BTN-FIDOFlight";
        internal const string ToolbarOabButtonID = "BTN-FIDOOAB";
        internal const string ToolbarKscButtonID = "BTN-FIDOKSC";

        public static ILogger logger;

        public KSPVessel current_vessel = new KSPVessel();

        static bool loaded = false;

        FidoWindow main_window = null;
        Fido.Mission.MissionVabWindow mission_vab_window = null;

        public override void OnPreInitialized()
        {
            logger = SWLogger;
            // Redux stable 0.2.8.5: unwrap the beta's late-bound UXML control data (see LateBoundCarrier).
            K2UI.LateBoundCarrier.Apply(new HarmonyLib.Harmony("FIDO.LateBoundCarrier"));
            // no autosaves during training + the campaign screen's "Clear autosaves" button
            Fido.Saves.SaveTidy.Apply(new HarmonyLib.Harmony("FIDO.SaveTidy"));
            // the VAB open dialog says which designs are trained
            Fido.Lift.Next.VabLoadDialogTag.Apply(new HarmonyLib.Harmony("FIDO.VabLoadDialog"));
        }

        /// <summary>
        /// Runs when the mod is first initialized.
        /// </summary>
        public override void OnInitialized()
        {

            Instance = this;
            // SWMetadata.Folder is a System.IO.DirectoryInfo, not a string (confirmed via the current
            // SpaceWarpPluginDescriptor's real field type) - string-concatenating it works today via
            // DirectoryInfo's implicit ToString(), but .FullName is the correct, intended accessor.
            //
            // AssetsLoader.Bundle = AssetBundle.LoadFromFile(SWMetadata.Folder.FullName + "/assets/bundles/k2d2_ui.bundle");
            //
            // Switched UI loading from the old prebuilt AssetBundle above to Redux's Addressables system -
            // see AssetsLoader.LoadUxml() for the new loading call and the reasoning. The line above is
            // commented out (not deleted) so reverting is a one-line job if Addressables doesn't pan out
            // in testing; k2d2_ui.bundle itself is left untouched on disk either way.

            // K2UIFactoryRegistration.RegisterAll() used to go here - a reflection-based workaround
            // that manually registered every K2UI custom control's legacy UxmlFactory with Unity's
            // internal VisualElementFactoryRegistry, needed because Unity's automatic factory scan
            // never recognized a BepInEx-loaded mod DLL as a "user assembly". That whole class is
            // gone now: every K2UI control moved to [UxmlElement]/[UxmlAttribute] (Unity 6.6 removes
            // UxmlFactory entirely, so this was happening either way), and the newer attribute-based
            // registration is handled by Redux itself for mod assemblies - no manual step needed.

            // K2D2 -> FIDO: first run copies the old k2d2_* files (see LegacyFiles)
            LegacyFiles.Migrate(AssemblyFolder, msg => logger.LogInfo(msg));

            var fidoPilotsMgr = new FidoPilotsMgr();
            SettingsFile.Init(this, SettingsPath);

            // Landing's Atmo/Vacuum profile split - two independent files, loaded up front here
            // (same as the main settings file above) so both are ready before LandingPilot's
            // constructor builds its two LandingSettings instances.
            SettingsFile.GetOrCreate("land_atmo", this, AtmoLandingSettingsPath);
            var landVacFile = SettingsFile.GetOrCreate("land_vac", this, VacLandingSettingsPath);

            // One-time migration: before this split, Landing's settings lived in the main file
            // (SettingsFile.Instance/fido_settings.json) under "land." keys - copy whatever's
            // already tuned there into the new VACUUM file specifically (not land_atmo), since
            // that's the profile that's actually been flown/tuned so far. CopyMissingKeys only
            // ever fills in keys land_vac doesn't already have, so this is a no-op on every launch
            // after the first (see its own comment in SettingsFile.cs) - never overwrites anything
            // tuned in land_vac since the split happened, and never touches land_atmo at all.
            int migrated = landVacFile.CopyMissingKeys(SettingsFile.Instance, "land.");
            if (migrated > 0)
                logger.LogInfo($"[FidoPlugin] Migrated {migrated} existing Landing setting(s) from the main settings file into fido_landing_vac.json.");

            gameObject.hideFlags = HideFlags.HideAndDontSave;
            DontDestroyOnLoad(gameObject);
            RegisterMessages();

            // create staging 
            new StagingPilot();

            // the Mission tab drives the pilots below
            pilots_manager.AddPilot(new Fido.Mission.MissionPilot());
            pilots_manager.AddPilot(new NodeExPilot());
            pilots_manager.AddPilot(new LiftPilot());
            // the Training tab drives Lift: pad -> orbit -> revert, X times
            pilots_manager.AddPilot(new Fido.Training.TrainingPilot());
            pilots_manager.AddPilot(new LandingPilot());
            pilots_manager.AddPilot(new DockingPilot());
            pilots_manager.AddPilot(new AttitudePilot());
            pilots_manager.AddPilot(new Fido.Constellation.ConstellationPilot());

            // pilots_manager.AddPilot(new DronePilot());
            // pilots_manager.AddPilot(new AttitudePilot());
            // pilots_manager.AddPilot(new LiftController());
     
            // controllerManager.AddController(new WarpController());
            // pilots_manager.AddPilot(new DockingAssist());

            // Load the UI from the asset bundle
            var myFirstWindowUxml = AssetsLoader.LoadUxml("K2D2_Window.uxml");

            // Create the window options object
            var windowOptions = new WindowOptions
            {
                // The ID of the window. It should be unique to your mod.
                WindowId = "FIDO",
                // Redux stable 0.2.8.5: follow the game's UI scale setting (WindowOptions.Default sets this;
                // an object initializer leaves it false, which selects UitkForKsp2's fixed-scale panel).
                UseStockScale = true,
                // The transform of parent game object of the window.
                // If null, it will be created under the main canvas.
                Parent = null,
                // Whether or not the window can be hidden with F2.
                IsHidingEnabled = true,
                // Whether to disable game input when typing into text fields.
                DisableGameInputForTextFields = true,
                MoveOptions = new MoveOptions
                {
                    // Whether or not the window can be moved by dragging.
                    IsMovingEnabled = false,
                    // Whether or not the window can only be moved within the screen bounds.
                    CheckScreenBounds = true
                }
            };

            // Create the window
            var fido_window = Window.Create(windowOptions, myFirstWindowUxml);
            // Add a controller for the UI to the window's game object
            main_window = fido_window.gameObject.AddComponent<FidoWindow>();

            // Register Flight AppBar button
            Appbar.RegisterAppButton(
                SWMetadata.Name,
                ToolbarFlightButtonID,
                FidoIcons.Icon,
                isOpen => main_window.IsWindowOpen = isOpen
            );

            // VAB: the mission plan editor
            mission_vab_window = Fido.Mission.MissionVabWindow.Create();
            Appbar.RegisterOABAppButton(
                "FIDO Mission",
                Fido.Mission.MissionVabWindow.ButtonId,
                FidoIcons.Icon,
                isOpen => mission_vab_window.IsOpen = isOpen
            );

            // Register OAB AppBar Button
            // Appbar.RegisterOABAppButton(
            //     ModName,
            //     ToolbarOabButtonID,
            //     AssetManager.GetAsset<Texture2D>($"{ModGuid}/images/icon.png"),
            //     isOpen => myFirstWindowController.IsWindowOpen = isOpen
            // );

            // Register KSC AppBar Button
            // Appbar.RegisterKSCAppButton(
            //     ModName,
            //     ToolbarKscButtonID,
            //     AssetManager.GetAsset<Texture2D>($"{ModGuid}/images/icon.png"),
            //     () => myFirstWindowController.IsWindowOpen = !myFirstWindowController.IsWindowOpen
            // );



            loaded = true;
        }

        public override void OnPostInitialized()
        {
        }

        // AssetsLoader.LoadUxml() (a separate static class, not a KerbalMod subclass) needs to call
        // Assets.LoadAssetAsync<T>() to load UI Toolkit assets via Addressables, but the base
        // KerbalMonoBehaviour.Assets property is `protected` - confirmed via CS0122 build error - so it's
        // not reachable from outside a KerbalMod subclass's own code, even through an instance reference
        // like FidoPlugin.Instance.Assets. This thin public wrapper exposes just the one call
        // AssetsLoader needs without loosening protection on anything else.
        public AsyncOperationHandle<T> LoadAddressableAsset<T>(string address)
        {
            return Assets.LoadAssetAsync<T>(address);
        }


        private static GameState[] validScenes = { GameState.FlightView, GameState.Map3DView };

        //private static GameState last_game_state ;

        private static bool ValidScene()
        {
            if (GeneralTools.Game?.GlobalGameState == null) return false;

            // FIXED during the Redux port verification pass: KSP.Game.GameStateMachine has no GetState()
            // method in the current Redux assemblies (confirmed via full member enumeration of
            // GameStateMachine in Assembly-CSharp.dll - no such member exists, and no unrelated overload
            // matches either). The real accessor is GetGameState(), which returns a GameStateConfiguration
            // struct whose public GameState field is what this code actually wants. Without this fix,
            // ValidScene() - which gates every per-frame pilot update - would throw a MissingMethodException
            // the instant it ran.
            KSP.Game.GameState state;
            try { state = GeneralTools.Game.GlobalGameState.GetGameState().GameState; }
            catch (System.NullReferenceException) { return false; } // game state not set up yet (early load)
            bool is_valid = validScenes.Contains(state);
            if (!is_valid)
            {
                ResetControllers();
            }
            return is_valid;
        }

        void Update()
        {
            // main_ui?.Update();

            Debug.developerConsoleVisible = false;
            // Update Models (even on non valid scenes)
            current_vessel.Update();

            // VAB stage-1 launch check on the marked separator (does nothing outside the VAB)
            Fido.Lift.Next.VabAdvisor.Update();
            Fido.Lift.Next.DebrisTracker.Update();

            /* TODO: Other mods interfacing
            if (FidoOtherModsInterface.instance == null)
            {
                var other_mods = new FidoOtherModsInterface();
                other_mods.CheckModsVersions();
            }
            */

            if (ValidScene())
            {
                // Debug.developerConsoleVisible = false;
                if (Input.GetKey(KeyCode.LeftAlt) && Input.GetKeyDown(KeyCode.O))
                    main_window.IsWindowOpen = !main_window.IsWindowOpen;

                StagingPilot.Instance.Update();

                if (!StagingPilot.Instance.is_staging)
                {
                    // Update Controllers only if staging is not in progress
                    pilots_manager.UpdateControllers();
                }
                Fido.KSPService.KerbalVoiceMute.Update(pilots_manager.AnyRunning());
            }
            else
            {
                Fido.KSPService.KerbalVoiceMute.Update(false);
                if (main_window != null && main_window.IsWindowOpen)
                    main_window.IsWindowOpen = false;
            }
        }

        // call on reset on controller, each on can reset it's status
        public static void ResetControllers()
        {
            if (!loaded) return;
            StagingPilot.Instance.onReset();
            Instance.pilots_manager.onReset(); 
        }

        public bool settings_visible = false;

        public PilotsManager pilots_manager = new PilotsManager();

        void FixedUpdate()
        {
            if (ValidScene())
            {
                pilots_manager.FixedUpdateControllers();
            }
        }

        private void LateUpdate()
        {
            if (ValidScene())
            {
                pilots_manager.LateUpdateControllers();
            }
        }

        private void RegisterMessages()
        {
            Game.Messages.Subscribe<GameStateChangedMessage>(msg =>
            {
                var message = (GameStateChangedMessage)msg;

                // if (message.CurrentState == GameState.FlightView)
                // {
                //     ShapeDrawer.Instance.can_draw = true;
                // }
                // else if (message.PreviousState == GameState.FlightView)
                // {
                //     ShapeDrawer.Instance.can_draw = false;
                // }
            });

            Game.Messages.Subscribe<VesselChangedMessage>(msg =>
            {
                var message = (VesselChangedMessage)msg;
                ResetControllers();
            });
        }

        // Public API to enable or disable a Pilot / Page
        [PublicAPI] public bool isPilotEnabled(string pilotName)
        {
            return FidoPilotsMgr.Instance.isPilotEnabled(pilotName);
        }

        [PublicAPI] public void EnableAllPilots(bool enabled)
        {
            FidoPilotsMgr.Instance.EnableAllPilots(enabled);
        }

        [PublicAPI] public void EnablePilot(string pilotName, bool enabled)
        {
            FidoPilotsMgr.Instance.EnablePilot(pilotName, enabled);
        }

        [PublicAPI] public List<string> GetPilotsNames()
        {
            return FidoPilotsMgr.Instance.GetPilotsNames();
        }

        // Public API to perform a precision node execution using FIDO
        [PublicAPI] public void FlyNode()
        {
            NodeExPilot.Instance.Start();
        }

        [PublicAPI] public void StopFlyNode()
        {
            NodeExPilot.Instance.Stop();
        }

        [PublicAPI] public bool IsFlyNodeRunning()
        {
            return NodeExPilot.Instance.isRunning;
        }

        // Public API to get the status of FIDO (used by FlightPlan)
        [PublicAPI] public string GetStatus()
        {
            return NodeExPilot.Instance.ApiStatus();
        }
    }
}
