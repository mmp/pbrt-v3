
# PBRT Exporter for Cinema 4D

## Compatibility

This version of the PBRT Exporter for Cinema 4D has been tested with PBRT v3 and Cinema 4D R16. It will probably also work with earlier and later Cinema 4D versions. It will definitely not work with different versions than PBRT v3.

## Installation

To install the exporter, just copy the 'PBRT Export' folder into the plugins folder of your Cinema 4D Installation. It will then show up in the Plugins menu the next time you start Cinema 4D.

## Operation

Choosing 'Export to PBRT...' from the Plugins menu will open the export dialog. The export mode controls where PBRT files are written and whether the renderer is started. The 'Render' mode will export the scene to a temporary location, start pbrt and open the resulting image in the Picture Viewer once the rendering is done. 'Export' will ask you for a location the pbrt scene should be written to. 'Export and Render' will ask you where the scene should be written and will start a rendering. For 'Render' and 'Export and Render' is important to let the plugin know where your pbrt executable is located. This can be specified using the 'Renderer' input field. 'Samples' allows you to specify the number of Samples per Pixel to be used. 'Light Intensity' lets you globally scale the intensity of all exported light sources in the scene. During export, a detailed log is created. The 'Logging Level' lets you choose how much detail you want to see in the log window at the bottom of the export dialog.

By default, the 'directlighting' integrator is used. When a Global Illumination effect is added to the regular Cinema 4D render settings, the 'path' integrator is used instead.

## Supported Features

- Omni and Distant Light sources are exported
- The Physical Sky object will have appropriate light sources added and the background is baked into an environment texture and added as infinite light.
- All geometric objects that create polygons are exported.
- The plugin attempts to move basic material attributes (base color, specularity, bump). Furthermore it detects translucency setups using the Backlight shader and attempts to translate those.

## Copyright

This plugin has been created by Burak Kahraman and Timm Dapper of Laubwerk GmbH (www.laubwerk.com). It is distributed under the same license as the rest of the PBRT repository.
