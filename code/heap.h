#ifndef __HEAP_H__
#define __HEAP_H__
#include <iostream>
#include <set>
#include <vector>
#include <climits>
#include <queue>

#include "typedef.h"

namespace OABusRouter
{
    
    struct HeapNode
    {
        int id;
        int depth;
        int backtrace;
        
        double CR;
        double PS;
        double CC;
        double CW;
        double CS;
        double EC;          // estimated cost
        vector<int> nodes;
        vector<int> xPrev;
        vector<int> yPrev;
        vector<int> xLast;
        vector<int> yLast;
        int xMin, yMin;
        int xMax, yMax;


        HeapNode()
        {}

        HeapNode(int numBits) :
            depth(0),
            backtrace(-1),
            CR(DBL_MAX),
            PS(DBL_MAX),
            CC(DBL_MAX),
            CW(DBL_MAX),
            CS(DBL_MAX),
            EC(DBL_MAX),
            xMin(INT_MAX),
            yMin(INT_MAX),
            xMax(INT_MIN),
            yMax(INT_MIN)
        {
            nodes = vector<int>(numBits, -1);
            xPrev = vector<int>(numBits, -1);
            yPrev = vector<int>(numBits, -1);
        }

        HeapNode(const HeapNode& node):
            id(node.id),
            depth(node.depth),
            backtrace(node.backtrace),
            CR(node.CR),
            PS(node.PS),
            CC(node.CC),
            CW(node.CW),
            CS(node.CS),
            EC(node.EC),
            xPrev(node.xPrev),
            yPrev(node.yPrev),
            xLast(node.xLast),
            yLast(node.yLast),
            xMin(node.xMin),
            yMin(node.yMin),
            xMax(node.xMax),
            yMax(node.yMax)
        {
            nodes.insert(nodes.end(), node.nodes.begin(), node.nodes.end());
        }

        double cost();
        
    };
    
    struct HeapNode3
    {
        int id;
        int depth;
        int backtrace;
        int numSegs;
        double EC;
        double PS;

        HeapNode3():
            id(-1),
            depth(INT_MAX),
            backtrace(-1),
            numSegs(INT_MAX),
            EC(0),
            PS(0)
        {}

        HeapNode3(const HeapNode3& node) :
            id(node.id),
            depth(node.depth),
            backtrace(node.backtrace),
            numSegs(node.numSegs),
            EC(node.EC),
            PS(node.PS)
        {}

        double cost()
        {
            EC = 0; //numSegs;
            //double total = 10*numSegs + PS;
            //double total = PS;
            double total = PS; //  + numSegs; // + PS;
            return (backtrace == -1) ? DBL_MAX : total; //depth + PS;
        };  
    };


    template <class A>
    struct Heap
    {
        template <class B>
        struct Comparison
        {
            bool operator() (B& n1, B& n2)
            {
                return (n1.EC > n2.EC) || ((n1.EC == n2.EC) && (n1.cost() > n2.cost()));
                //return n1.cost() > n2.cost();
            };


            //bool operator() (HeapNode& n1, HeapNode& n2)
            //{
                //return n1.EC + 2*n1.cost() > n2.EC + 2*n2.cost();
                //return n1.cost() > n2.cost();
            //}

        };

        
        //priority_queue<HeapNode, vector<HeapNode>, Comparison> PQ;
        priority_queue<A, vector<A>, Comparison<A>> PQ;

        // member functions
        void push(vector<A> &next)
        {
            for(int i=0; i < next.size(); i++)
                PQ.push(next[i]);
        }

        void push(A &next)
        {
            PQ.push(next);   
        }
        
        void pop()
        {
            PQ.pop();
        }
        
        bool empty()
        {
            return PQ.empty();   
        }
        
        int size()
        {
            return PQ.size();
        }
        
        A top()
        {
            return PQ.top();      
        }

    };
};
#endif

