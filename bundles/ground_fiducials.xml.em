@{PLANE_WIDTH = 10.0*0.3048*100}
@{PLANE_LENGTH = 20.0*0.3048*100}
@{FIDUCIAL_SIZE = 40}
@{FIDUCIAL_Y = [
	+0*PLANE_WIDTH/6,
	+2*PLANE_WIDTH/6,
	+4*PLANE_WIDTH/6
]}
@{FIDUCIAL_X = [
	+0*PLANE_LENGTH/8,
	+2*PLANE_LENGTH/8,
	+4*PLANE_LENGTH/8,
	+6*PLANE_LENGTH/8
]}

@{F = FIDUCIAL_SIZE}
<?xml version="1.0" encoding="UTF-8" standalone="no" ?>
<multimarker markers="@(len(FIDUCIAL_X)*len(FIDUCIAL_Y))">
	@[def marker(n, x, y)]
	<marker index="@(n)" status="2">
		<corner x="@(x - F/2)" y="@(y - F/2)" z="0.0" />
		<corner x="@(x + F/2)" y="@(y - F/2)" z="0.0" />
		<corner x="@(x + F/2)" y="@(y + F/2)" z="0.0" />
		<corner x="@(x - F/2)" y="@(y + F/2)" z="0.0" />
	</marker>
	@[end def]

	@{n_markers = 0}
	@[for y in FIDUCIAL_Y]
	@[for x in FIDUCIAL_X]
	@( marker(n_markers, x, y) )
	@{n_markers += 1}
	@[end for]
	@[end for]
</multimarker>
