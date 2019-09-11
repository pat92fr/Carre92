1) blender to gltf
https://discourse.panda3d.org/t/yabee-and-blender-2-8/24996/16
Blender 2.80 glTF exporter. I recommend using the following settings:
	Set Format to glTF Embeded or glTF Separate (Binary glTF is not currently supported by panda3d-gltf)
	Enable Custom Properties to export get tags (game properties are no longer supported)
	Disable +Y Up and tell panda3d-gltf to skip axis conversion (axis conversion is still a WIP in panda3d-gltf)
	Enable Punctual Lights (Objects tab) to get lights
	Select Always Sample Animations (Animation tab)

2) gltf2bam CLtool
https://github.com/Moguri/panda3d-gltf

3) place media to directory without 'space' into full path name