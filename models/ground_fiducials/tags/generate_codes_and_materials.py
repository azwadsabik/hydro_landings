#!/usr/bin/env python
import os
for i in xrange(0,12):
    os.system("rosrun ar_track_alvar createMarker %d" % i) #<1>
    fn = "MarkerData_{0}.png".format(i)
    #os.system("convert {0} -bordercolor white -border 100x100 {0}".format(fn)) #<2>
    with open("fiducial_%d.material" % i, 'w') as f: #<3>
      f.write("""
material fiducial_%d {
  receive_shadows on
  technique {
    pass {
      ambient 1.0 1.0 1.0 1.0
      diffuse 1.0 1.0 1.0 1.0
      specular 0.5 0.5 0.5 1.0
      lighting on
      shading gouraud
      texture_unit { texture MarkerData_%d.png }
    }
  }
}
""" % (i, i))
