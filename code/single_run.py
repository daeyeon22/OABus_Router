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

benchDirList = ["../bench"]
dirpos = "../bench"
binaryName = "./iccad18obr"
evalpos = "../eval/eval_1.0-a7"
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


if len(sys.argv) <=1:
    print("usage:   ./single_run.py <benchname or number> ")
    print("Example: ")
    print("         ./single_run.py 4 ")
    print("         ./single_run.py example_2 ")
    sys.exit(1)

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

curTime = datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
if len(sys.argv) > 2:
    exeEval = (sys.argv[2] == "eval")
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
    exeStr = "%s -input %s/%s.input -output %s/%s.out | tee %s/%s_%s.log" % (binaryName, dirpos, benchName, outpos, benchName, logpos, benchName, curTime)
    ExecuteCommand(exeStr)
    endTime = time.time();
    evalStr = "%s/%s %s/%s.input %s/%s.out | tee %s/%s.eval" % (evalpos, evaluator, dirpos, benchName, outpos, benchName, logpos, benchName)
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
