@{PI = 3.141592653589793}
@{PLANE_WIDTH = 10.0*0.3048}
@{PLANE_LENGTH = 20.0*0.3048}
@{FIDUCIAL_SIZE = 0.5*0.3048}
@{FIDUCIAL_X = [
	-2*PLANE_WIDTH/6,
	0,
	+2*PLANE_WIDTH/6
]}
@{FIDUCIAL_Y = [
	-3*PLANE_LENGTH/8,
	-1*PLANE_LENGTH/8,
	+1*PLANE_LENGTH/8,
	+3*PLANE_LENGTH/8
]}

<?xml version='1.0'?>
<sdf version ='1.4'>
	<model name ='ground_fiducials'>
	<static>true</static>
		<link name ='ground'>
			
			@[def fiducial(n, x, y)]
			<visual name="fiducial_@(n)">
			<pose>@(x) @(y) 0 0 0 @(PI)</pose>
			<geometry><box><size>
				@(FIDUCIAL_SIZE) @(FIDUCIAL_SIZE) 0.02
			</size></box></geometry>
			<material><script>              
				<uri>model://ground_fiducials/tags</uri>
				<name>fiducial_@(n)</name>
			</script></material>
			</visual>
			@[end def]
			
			@{n_fiducials = 0}
			@[for x in FIDUCIAL_X]
			@[for y in FIDUCIAL_Y]
			@( fiducial(n_fiducials, x, y) )
			@{n_fiducials += 1}
			@[end for]
			@[end for]
		</link>

	</model>

</sdf>
