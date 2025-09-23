# Placeholder for Fusion 360 add-in
import adsk.core, adsk.fusion, adsk.cam
import os
import sys

def run(context):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        ui.messageBox('Impellor V3 Add-in: Select input mesh and configure settings')
        # TODO: Implement file picker for .stl/.obj/.ply
        # TODO: Implement settings UI for blade count, ICP parameters
        # TODO: Call cli_engine/fit.exe with config.json
    except Exception as e:
        ui = adsk.core.Application.get().userInterface
        ui.messageBox(f'Failed to run add-in: {str(e)}')

def stop(context):
    try:
        ui = adsk.core.Application.get().userInterface
        ui.messageBox('Impellor V3 Add-in stopped')
    except:
        pass
