-- FIDO Mission: every command part (pod or probe core) can carry the craft's mission plan.
-- In the VAB its part window gets a "Carries the mission" toggle and an "Edit mission" button;
-- the plan itself is edited in the FIDO Mission window (VAB) or the Mission tab (flight).

local function makeVisualsOverride()
    return {
        PartComponentModuleName = "PartComponentModule_FidoMission",
        ModuleDisplayName = "PartModules/FidoMission/Name",
        ShowHeader = true,
        ShowFooter = true,
    }
end

PM.Parts:Patch("FIDO_Mission")
        :Has("Module_Command")
        :Do(function(part)
            part:AddModule("Module_FidoMission", function(module)
                module:AddData("Data_FidoMission", function(data) end)
            end)
            if part.PAMModuleVisualsOverride ~= nil then
                part.PAMModuleVisualsOverride:Append(makeVisualsOverride())
            else
                part.PAMModuleVisualsOverride = { makeVisualsOverride() }
            end
        end)
