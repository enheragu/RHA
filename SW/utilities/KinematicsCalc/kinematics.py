# @Author: Enrique Heredia Aguado <quique>
# @Date:   29-Oct-2017
# @Project: RHA
# @Last modified by:   quique
# @Last modified time: 29-Oct-2017

#work based on Obijuan example: https://github.com/Obijuan/friki

import sys
sys.path.append("/usr/lib/freecad/lib/")
import FreeCAD

from FreeCAD import Vector
from pyooml import *

FreeCAD.newDocument('Sans nom')
FreeCAD.setActiveDocument('Sans_nom')
doc = FreeCAD.ActiveDocument

import Arch
Arch.makeStructure(length=1000.0, width=100.0, height=100.0, name="Poutre")
doc.recompute()
doc.saveAs(str("Test.fcstd"))

a1 = 60
a2 = 70
l1 = 40
l2 = 40
v1 = Vector(l1, 0, 0)
v2 = Vector(l2, 0, 0)

f0 = frame()
f1 = frame()
f2 = frame()
sv1 = svector(v1).color("yellow")
sv2 = svector(v2).color("yellow")

import HMatrix

Ma = HMatrix.Roty(a1)
Mb = HMatrix.Translation(v1)
Mc = HMatrix.Roty(a2)
Md = HMatrix.Translation(v2)


sv1.T = Ma
f1.T = Ma * Mb
