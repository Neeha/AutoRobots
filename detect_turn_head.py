from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import core
import pose
import commands
import cfgstiff
from task import Task
from state_machine import Node, C, T, S, StateMachine

class Playing(StateMachine):    

    class Turn_Head(Node):
        def run(self):
            commands.setHeadPanTilt(2,20)
            self.postSignal("Ball detected and head turn") 
   
    def run(self):
        turn_head = self.Turn_Head()        
        ball = memory.world_objects.getObjPtr(core.WO_BALL)
        if ball.seen:
            print("ball seen")
            turn_head = self.Turn_Head()
        robot = memory.world_objects.getObjPtr(cache_.robot_state.WO_SELF);
        print ("robot location: ",robot.loc)
        print ("robot orientation: ",robot.orientation)
        self.trans(turn_head, S("done"))
        
    