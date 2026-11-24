-- FIDO Mission part actions: every part a mission step can drive (fairings, decouplers, docking
-- ports, lights) gets an "Add action to mission" button in its VAB part window. The tag it hands
-- out is saved on the part, so the step finds the same part again in flight.

local actionable = { "Module_Fairing", "Module_Decouple", "Module_DockingNode", "Module_Light" }

local function makeVisualsOverride()
    return {
        PartComponentModuleName = "PartComponentModule_FidoActionTarget",
        ModuleDisplayName = "PartModules/FidoActions/Name",
        ShowHeader = true,
        ShowFooter = true,
    }
end

PM.Parts:Patch("FIDO_ActionTarget")
        :Do(function(part)
            local any = false
            for _, m in ipairs(actionable) do
                if part:HasModule(m) then any = true end
            end
            if not any then return end
            part:AddModule("Module_FidoActionTarget", function(module)
                module:AddData("Data_FidoActionTarget", function(data) end)
            end)
            if part.PAMModuleVisualsOverride ~= nil then
                part.PAMModuleVisualsOverride:Append(makeVisualsOverride())
            else
                part.PAMModuleVisualsOverride = { makeVisualsOverride() }
            end
        end)
