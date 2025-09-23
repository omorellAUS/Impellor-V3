# Fusion 360 Add-in for Impeller Reverse-Engineering (v3)
# Supports .stl, .obj, .ply; includes ICP, blade segmentation, surface fitting, gap repair, fillets, parameter reports

import adsk.core, adsk.fusion, traceback
import subprocess
import json
import os

app = adsk.core.Application.get()
ui = app.userInterface
cmd_def = None

class ReverseEngCommand(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        event_args = adsk.core.CommandEventArgs.cast(args)
        cmd = event_args.command
        inputs = cmd.commandInputs

        if event_args.firingEvent == cmd.commandDefinition.commandCreated:
            self.on_create(inputs)
        elif event_args.firingEvent == cmd.commandDefinition.execute:
            self.on_execute(inputs)
        elif event_args.firingEvent == cmd.commandDefinition.destroy:
            global cmd_def
            cmd_def = None

    def on_create(self, inputs):
        inputs.addStringValueInput('mesh_path', 'Main Mesh File', '')
        inputs.addValueInput('tolerance', 'Fitting Tolerance (mm)', 'mm', adsk.core.ValueInput.createByReal(0.02))
        inputs.addIntegerSpinnerInput('blade_count', 'Number of Blades', 1, 100, 1, 8)
        inputs.addValueInput('fillet_radius', 'Fillet Radius (mm)', 'mm', adsk.core.ValueInput.createByReal(0.5))
        inputs.addValueInput('section_count', 'Blade Sections', '', adsk.core.ValueInput.createByReal(8))

    def on_execute(self, inputs):
        try:
            mesh_path = inputs.itemById('mesh_path').value
            if not mesh_path:
                mesh_path = ui.openFile('Select Main Mesh File', '', 'Mesh Files (*.stl;*.obj;*.ply)')
                if not mesh_path:
                    ui.messageBox('No file selected')
                    return

            tolerance = inputs.itemById('tolerance').value
            blade_count = inputs.itemById('blade_count').value
            fillet_radius = inputs.itemById('fillet_radius').value
            section_count = int(inputs.itemById('section_count').value)

            config_path = os.path.join(os.path.dirname(__file__), '../config.json')
            with open(config_path, 'r') as f:
                config = json.load(f)
            cli_path = config['engine_path']
            mesh_paths = config.get('mesh_paths', [mesh_path])

            output_dir = os.path.join(os.path.dirname(__file__), 'output')
            os.makedirs(output_dir, exist_ok=True)
            step_path = os.path.join(output_dir, 'pocket.step')
            json_path = os.path.join(output_dir, 'output.json')
            cli_config = {
                'mesh_paths': mesh_paths,
                'tolerance': tolerance,
                'blade_count': blade_count,
                'fillet_radius': fillet_radius,
                'section_count': section_count
            }
            cli_config_path = os.path.join(output_dir, 'cli_config.json')
            with open(cli_config_path, 'w') as f:
                json.dump(cli_config, f)

            result = subprocess.run([cli_path, '--config', cli_config_path, '--output_step', step_path, '--output_json', json_path], capture_output=True, text=True)
            if result.returncode != 0:
                ui.messageBox(f'CLI failed: {result.stderr}')
                return

            with open(json_path, 'r') as f:
                output_data = json.load(f)
            axis_vector = output_data.get('axis_vector')

            design = adsk.fusion.Design.cast(app.activeProduct)
            import_manager = design.importManager
            import_options = import_manager.createSTEPImportOptions(step_path)
            import_manager.importToTarget(import_options, design.rootComponent)

            imported_body = design.rootComponent.bRepBodies.item(design.rootComponent.bRepBodies.count - 1)

            if not imported_body.isSolid:
                stitch_input = design.rootComponent.features.stitchFeatures.createInput(imported_body.faces, adsk.core.ValueInput.createByReal(tolerance / 10))
                stitch_input.operation = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
                design.rootComponent.features.stitchFeatures.add(stitch_input)

            if axis_vector:
                origin = adsk.core.Point3D.create(0, 0, 0)
                direction = adsk.core.Vector3D.create(axis_vector.get('x', 0), axis_vector.get('y', 0), axis_vector.get('z', 1))
                axis_line = adsk.core.InfiniteLine3D.create(origin, direction)
                axes = design.rootComponent.constructionAxes
                axis_input = axes.createInput()
                axis_input.setByLine(axis_line)
                bore_axis = axes.add(axis_input)

            if blade_count > 1 and imported_body.isSolid:
                pattern_input = design.rootComponent.features.circularPatternFeatures.createInput(
                    adsk.core.ObjectCollection.create().add(imported_body), bore_axis)
                pattern_input.quantity = adsk.core.ValueInput.createByReal(blade_count)
                pattern_input.isSymmetric = True
                pattern_feature = design.rootComponent.features.circularPatternFeatures.add(pattern_input)

                combine_input = design.rootComponent.features.combineFeatures.createInput(
                    pattern_feature.bodies.item(0), pattern_feature.bodies)
                combine_input.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
                design.rootComponent.features.combineFeatures.add(combine_input)

            if fillet_radius > 0:
                edge_collection = adsk.core.ObjectCollection.create()
                for face in imported_body.faces:
                    for edge in face.edges:
                        edge_collection.add(edge)
                fillet_input = design.rootComponent.features.filletFeatures.createInput()
                fillet_input.addConstantRadiusEdgeSet(edge_collection, adsk.core.ValueInput.createByReal(fillet_radius), True)
                design.rootComponent.features.filletFeatures.add(fillet_input)

            ui.messageBox('Processing complete! Check output folder for deviation map and parameter report.')

        except:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def run(context):
    try:
        global cmd_def
        cmd_def = ui.commandDefinitions.addButtonDefinition(
            'reverseEngCmd', 'Impeller Reverse Engineer', 'Run reverse engineering (v3)', '')
        cmd_def.commandCreated.add(ReverseEngCommand())
        workspace = ui.workspaces.itemById('FusionSolidEnvironment')
        toolbar = workspace.toolbarPanels.itemById('SolidScriptsAddinsPanel')
        control = toolbar.controls.addCommand(cmd_def)
        control.isPromoted = True
    except:
        ui.messageBox('Failed to run add-in:\n{}'.format(traceback.format_exc()))

def stop(context):
    try:
        global cmd_def
        if cmd_def:
            cmd_def.deleteMe()
    except:
        ui.messageBox('Failed to stop add-in:\n{}'.format(traceback.format_exc()))
