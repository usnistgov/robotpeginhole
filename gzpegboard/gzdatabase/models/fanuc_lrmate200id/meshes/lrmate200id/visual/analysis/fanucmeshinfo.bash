#!/bin/bash
python38="/c/Users/michalos/Documents/bin/Python/PyAnalyzeMeshes/venv/Scripts/python.exe"
bin=/C/Users/michalos/Documents/bin/Python/PyAnalyzeMeshes
stl="/C/opt/ros/noetic/gztaskboard\gzdatabase/models/fanuc_lrmate200id/meshes/lrmate200id/visual/analysis"

$python38 $bin/AnalyzeMeshes.py  -i "$stl/base_link.stl" 

$python38 $bin/AnalyzeMeshes.py -i "$stl/link_1.stl" 

$python38 $bin/AnalyzeMeshes.py -i "$stl/link_3.stl" 

$python38 $bin/AnalyzeMeshes.py -i "$stl/link_4.stl" 

$python38 $bin/AnalyzeMeshes.py -i "$stl/link_5.stl" 

$python38 $bin/AnalyzeMeshes.py -i "$stl/link_6.stl" 



