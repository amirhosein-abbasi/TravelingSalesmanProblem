from random import random
import math
from Problem import *
from xmlrpc.client import MAXINT


TSP = Problem()
TSP.input()


class Algorithms :

    nodeVisited=0
    nodeExpanded=0

    def hillClimbing_randomRestart(self,problem):
        global TSP

        minCost = MAXINT
        bestPath = ""


        currentState = problem.initial()
        bestNeighbor = None
        bestCostNeighbour = problem.heuristic(currentState)

        iteration = 20

        while (iteration > 0)  :
            print("***********************")
            print("we go to the node : " ,currentState.name,"with the cost : ",problem.heuristic(currentState))
            haveBetter = 0
            for neiAct in problem.actions(currentState) :
                result=problem.result(currentState,neiAct)
                self.nodeVisited=self.nodeVisited+1
                print("the neighbour is :",result.name,"the cost is :",problem.heuristic(result))
                if(problem.heuristic(result) < bestCostNeighbour ):
                    bestCostNeighbour = problem.heuristic(result)
                    bestNeighbor = result
                    haveBetter=1
                    if(bestCostNeighbour < minCost):
                        minCost = bestCostNeighbour
                        bestPath = bestNeighbor
            if(haveBetter==0):
                print("we have reached an optimized place in : ",currentState.name,"with the cost : ",problem.heuristic(currentState))
                currentState = problem.randomState()
                self.nodeExpanded = self.nodeExpanded  + 1
                bestNeighbor = None
                bestCostNeighbour = problem.heuristic(currentState)
                print("we start again from : ", currentState.name, "with the cost : ",problem.heuristic(currentState))
            iteration=iteration-1

        print("the best path is : ",bestPath.name,"with the cost : ",minCost)
        print("node visited :",self.nodeVisited )
        print("node expanded :",self.nodeExpanded)


    def hillClimbing_standard (self,problem) :
        global TSP

        minCost = MAXINT
        bestPath = ""


        currentState = problem.initial()
        bestNeighbor = None
        bestCostNeighbour = problem.heuristic(currentState)

        while (1==1)  :
            print("***********************")
            print("we go to the node : " ,currentState.name,"with the cost : ",problem.heuristic(currentState))
            haveBetter = 0
            for neiAct in problem.actions(currentState) :
                result=problem.result(currentState,neiAct)
                self.nodeVisited=self.nodeVisited+1
                print("the neighbour is :",result.name,"the cost is :",problem.heuristic(result))
                if(problem.heuristic(result) < bestCostNeighbour ):
                    bestCostNeighbour = problem.heuristic(result)
                    bestNeighbor = result
                    haveBetter=1
            if(haveBetter==0):
                print("we have reached an optimized place in : ",currentState.name,"with the cost : ",problem.heuristic(currentState))
                print("node visited :", self.nodeVisited)
                print("node expanded :", self.nodeExpanded)
                break
            currentState = bestNeighbor
            self.nodeExpanded=self.nodeExpanded+1

    def hillClimbing_firstChoice(self,problem):
        global TSP

        minCost = MAXINT
        bestPath = ""

        currentState = problem.initial()

        bestNeighbor = None
        bestCostNeighbour =problem.heuristic(currentState)


        while (1==1)  :
                print("***********************")
                print("we ARE in the node : ", currentState.name, "with the cost : ", problem.heuristic(currentState))

                randAct = problem.randomAction(currentState)
                result = problem.result(currentState, randAct)
                self.nodeVisited=self.nodeVisited+1
                result.visited=1

                haveBetter=0
                haveUnvisited = 0

                for act in problem.actions(currentState):
                    if(problem.result(currentState,act).visited==0):
                        haveUnvisited=1
                if haveUnvisited==0 :
                    print ("we have reached an optimized place in : ",currentState.name,"with the cost : ",problem.heuristic(currentState))
                    print("node visited :", self.nodeVisited)
                    print("node expanded :", self.nodeExpanded)
                    return
                if(problem.heuristic(result) < bestCostNeighbour ):
                        bestCostNeighbour = problem.heuristic(result)
                        currentState = result
                        self.nodeExpanded=self.nodeExpanded+1
                        print("we GO to the node : ",currentState.name,"with the cost : ",problem.heuristic(currentState))
                        haveBetter=1
                if(haveBetter==1):
                        continue











    def simulated_annealing(self,problem):
        global TSP

        minCost = MAXINT
        bestPath = ""

        T=1.0
        T_min=0.001
        method=2
        # method can be 0 or 1 or 2 ... for choosing the way of decreasing T
        alpha = 3
        TSP=problem
        currentState = TSP.initial()


        while T>T_min :
            i=1
            while i<50 :
                 nextState=TSP.result(currentState,TSP.randomAction(currentState))
                 self.nodeVisited=self.nodeVisited+1
                 if TSP.heuristic(nextState) < TSP.heuristic(currentState) :
                     currentState = nextState
                     self.nodeExpanded=self.nodeExpanded+1
                     print(currentState.name)
                     if (TSP.heuristic(currentState) < minCost):
                         minCost = TSP.heuristic(currentState)
                         bestPath = currentState.name
                 elif TSP.heuristic(nextState) >= TSP.heuristic(currentState) :
                     P=self.calculateP(T,TSP.heuristic(currentState),TSP.heuristic(nextState))
                     if( P > random.random()) :
                         currentState = nextState
                         self.nodeExpanded=self.nodeExpanded+1
                         print(currentState.name)
                         if(TSP.heuristic(currentState) <minCost):
                             minCost = TSP.heuristic(currentState)
                             bestPath = currentState.name
                 i+=1
            T = self.decreaseT(T, alpha , method)


        print ("best path is :" ,bestPath ," with cost : " ,minCost )
        print("node visited :", self.nodeVisited)
        print("node expanded :", self.nodeExpanded)

    # def randomSelect (self,state):
    #     global TSP
    #     i=int(random()*len(TSP.actions(state)))
    #     return TSP.result(state,TSP.actions(state)[i]) ;

    def decreaseT(self ,T,alpha,method) :
        if(method==0):
            T=T*alpha
            return T
        elif(method==1):
            T=T-alpha
            return T
        elif (method==2):
            T=T/alpha
            return T
    def calculateP(self,T,oldCost,newCost):
        if(newCost>oldCost):
            return 1.0
        else :
            dif =newCost - oldCost
        return math.exp(-abs(dif)/T)

alg=Algorithms()
alg.hillClimbing_randomRestart(TSP)
