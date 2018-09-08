#!/usr/bin/python

# 
# This script written by Mgwoo.
# 2018.02.22.
# 
# mgwoo@unist.ac.kr

import os
import sys
import subprocess as sp
import time
from datetime import datetime

benchDirList = ["../bench", "../bench_big"]
dirpos = "../bench"
binaryName = "./iccad18obr"
evalpos = "../eval/eval_1.0-a3"
evaluator = "eval"
outpos = "../output"
logpos = "../log"

def Tokenizer( line, delim ):
    vals = line.split()
    vals = [ elem for elem in vals if elem not in delim ]
    return vals


def ReadScore( benchName, outputFile ):
    inFile = open(outputFile, 'r')
    lines = inFile.readlines()
    score = {}
    target = [ 'CR', 'Ps', 'Pf', 'cost' ]

    for i in range(len(lines)):
        line = lines[i]
        delim = [ '[', ']', '=', '+', '*' ]
        vals = Tokenizer(line, delim)

        if vals[2] in target:
            score[vals[2]] = vals[len(vals)-1]

    return score

def ExecuteCommand( curCmd ):
    print( curCmd )
    sp.call( curCmd , shell=True)


if len(sys.argv) <=2:
    print("usage:   ./run.py <benchname or number> <# Threads>")
    print("Example: ")
    print("         ./run.py 4 1")
    print("         ./run.py example_2 1")
    sys.exit(1)

if(len(sys.argv) >4):
    if(sys.argv[4] == "big"):
        dirpos = benchDirList[1]
    




benchNum = -1
benchName = ""
benchList = []
benchList = sorted(os.listdir(dirpos))
runAll = False
if sys.argv[1].isdigit():
    benchNum = int(sys.argv[1])
    benchName = benchList[benchNum]
elif sys.argv[1] == "all":
    runAll = True
else:
    benchName = sys.argv[1]

numThreads = int(sys.argv[2])
curTime = datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
if len(sys.argv) > 3:
    exeEval = (sys.argv[3] == "eval")
else:
    exeEval = False



if runAll == False: 
    #benchName = benchList[benchNum]
    benchList = []
    benchList.append(benchName)

if exeEval == True and runAll == True:
    sumFile = open("%s/summarize.txt" % (logpos), 'w')

for fileName in benchList:
    benchName = fileName.split('.')[0]
    #print curTime

    rmlogStr = "rm %s/%s_*" % (logpos, benchName)
    ExecuteCommand(rmlogStr)
    startTime = time.time();
    exeStr = "%s -input %s/%s.input -threads %d -output %s/%s.out | tee %s/%s_%s.log" % (binaryName, dirpos, benchName, numThreads, outpos, benchName, logpos, benchName, curTime)
    ExecuteCommand(exeStr)
    endTime = time.time();
    evalStr = "%s/%s %s/%s.input %s/%s.out | tee %s/%s.eval" % (evalpos, evaluator, dirpos, benchName, outpos, benchName, logpos, benchName)
    #evalStr = "%s/%s %s/%s.input %s/%s.out > %s/%s.eval" % (evalpos, evaluator, dirpos, benchName, outpos, benchName, logpos, benchName)
    if exeEval == True:
        ExecuteCommand(evalStr)
        evalFile = "%s/%s.eval" % (logpos, benchName)
        score = ReadScore( benchName, evalFile )
        if runAll == True:
            sumFile.write( "< %s >\n" % (benchName) )
            for key, value in score.items():
                sumFile.write( "%4s :   %7s\n" % (key, value) )
            sumFile.write( "%4s :   %3.4f(s)\n" % ("RT", endTime - startTime) )
            sumFile.write( "\n\n" )

if exeEval == True and runAll == True:
    sumFile.close()


'''
if len(sys.argv) > 4:
    exeGprof = (sys.argv[4] == "runtime")
else:
    exeGprof = False
'''


'''
gprofStr = "gprof %s gmon.out > ./gprof/%s_result.txt" % (binaryName, benchName)
if exeGprof == True:
    ExecuteCommand(gprofStr)
'''
