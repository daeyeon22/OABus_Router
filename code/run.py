#!/usr/bin/python

# 
# This script written by Mgwoo.
# 2018.02.22.
# 
# mgwoo@unist.ac.kr

import os
import sys
import subprocess as sp
from datetime import datetime


dirpos = "../bench"
binaryName = "./iccad18obr"
evalpos = "../eval/eval_1.0-a3"
evaluator = "eval"
outpos = "../output"
logpos = "../log"

def ExecuteCommand( curCmd ):
    print( curCmd )
    sp.call( curCmd , shell=True)


if len(sys.argv) <=2:
    print("usage:   ./run.py <benchname or number> <# Threads>")
    print("Example: ")
    print("         ./run.py 4 1")
    print("         ./run.py example_2 1")
    sys.exit(1)


benchNum = -1
benchName = ""
if sys.argv[1].isdigit():
    benchNum = int(sys.argv[1])
    benchName = sorted(os.listdir(dirpos))[benchNum]
elif sys.argv[1] == "all":
    benchName = sorted(os.listdir(dirpos))
else:
    benchName = sys.argv[1]

numThreads = int(sys.argv[2])
curTime = datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
benchName = benchName.split('.')[0]
if len(sys.argv) > 3:
    exeEval = (sys.argv[3] == "eval")
else:
    exeEval = False

if len(sys.argv) > 4:
    exeGprof = (sys.argv[4] == "runtime")
else:
    exeGprof = False

#print curTime

rmlogStr = "rm %s/*" % (logpos)
ExecuteCommand(rmlogStr)
exeStr = "%s -input %s/%s.input -threads %d -output %s/%s.out | tee %s/%s_%s.log" % (binaryName, dirpos, benchName, numThreads, outpos, benchName, logpos, benchName, curTime)
ExecuteCommand(exeStr)
evalStr = "%s/%s %s/%s.input %s/%s.out | tee %s/%s.eval" % (evalpos, evaluator, dirpos, benchName, outpos, benchName, logpos, benchName)
if exeEval == True:
    ExecuteCommand(evalStr)

gprofStr = "gprof %s gmon.out > ./gprof/%s_result.txt" % (binaryName, benchName)
if exeGprof == True:
    ExecuteCommand(gprofStr)






