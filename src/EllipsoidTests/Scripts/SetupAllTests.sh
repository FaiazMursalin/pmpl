#!/bin/bash

#NOTE: to be run from the Scripts directory!
if [ ! -f xmlgen ]; then
    make
fi
if [ ! -f averageCols ]; then
    g++ averageCols.cpp -o averageCols
fi


# Go to Tests directory:
cd ../Tests

#Pentomino:
cd Pentomino
../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml

#Constrained Pentomino:
cd ../ConstrainedPentomino
../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml


#Frame:
cd ../Frame
../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml


#BoxPegs:
cd ../BoxPegs
../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml

cd ../Gear
../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml

cd ../ToyPlane
../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml

#Drill test
cd ../Drill
../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml


# stack puzzle:
cd ../StackPuzzle3-2D-Translational
../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml

#Gearbox no subs:
cd ../Gearbox-noSubs
../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml

#Gearbox:
cd ../Gearbox
../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml


#Gearbox no subs:
cd ../EngineFewerScrews
../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml


cd ../Rotational

cd SimpleCoax
../../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml

cd ../StackPuzzle3-2D
../../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml


#Subassembly disassembly in parallel test:
cd ../../SubassemblyParallel

cd Frame
../../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml

cd ../ConstrainedPentomino
../../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml

cd ../Drill
../../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml

cd ../Gear
../../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml

cd ../ToyPlane
../../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml

cd ../Pentomino
../../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml

cd ../StackPuzzle3-2D-Translational
../../../Scripts/xmlgen -x Master.xml -n 10 -s 12345678 -t experiments.txt -o tests.xml

cd ../..


#NOTE: The Exhaustive tests don't need setup since they are single runs


