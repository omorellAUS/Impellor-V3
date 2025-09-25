--- fusion_addin/main.py
+++ fusion_addin/main.py
@@ -1,5 +1,6 @@
 import adsk.core, adsk.fusion
 import os, json

 def run(context):
     ui = adsk.core.UserInterface.cast(None)
     try:
         app = adsk.core.Application.get()
         ui = app.userInterface

         # Load config.json
         with open("config.json", "r") as f:
             config = json.load(f)
-        blade_count = config["blade_count"]
+        blade_count_min = config.get("blade_count_min", 8)
+        blade_count_max = config.get("blade_count_max", 20)
+
+        # Example: UI input for blade count
+        blade_input = ui.inputBox("Enter blade count (8-20)", "Blade Count", "12")
+        if blade_input[0]:
+            blade_count = int(blade_input[1])
+            if blade_count < blade_count_min or blade_count > blade_count_max:
+                ui.messageBox(f"Blade count must be between {blade_count_min} and {blade_count_max}")
+                return
+        else:
+            ui.messageBox("Blade count input cancelled")
+            return

         # Run fit.exe with config.json
         os.system(f"fit.exe config.json")
         # Import STEP, stitch, pattern, etc.
         # ...
     except:
         ui.messageBox("Failed: {}".format(traceback.format_exc()))
