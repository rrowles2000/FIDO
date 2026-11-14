-- K2-D2 Mission: every command part (pod or probe core) can carry the craft's mission plan.
-- In the VAB its part window gets a "Carries the mission" toggle and an "Edit mission" button;
-- the plan itself is edited in the K2-D2 Mission window (VAB) or the Mission tab (flight).

local function makeVisualsOverride()
    return {
        PartComponentModuleName = "PartComponentModule_K2Mission",
        ModuleDisplayName = "PartModules/K2D2Mission/Name",
        ShowHeader = true,
        ShowFooter = true,
    }
end

PM.Parts:Patch("K2D2_Mission")
        :Has("Module_Command")
        :Do(function(part)
            part:AddModule("Module_K2Mission", function(module)
                module:AddData("Data_K2Mission", function(data) end)
            end)
            if part.PAMModuleVisualsOverride ~= nil then
                part.PAMModuleVisualsOverride:Append(makeVisualsOverride())
            else
                part.PAMModuleVisualsOverride = { makeVisualsOverride() }
            end
        end)
