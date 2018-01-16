from threading import Timer
from transitions import Machine, State, Transition

# github.com/pytransitions/transitions

class TenderBotStateMachine(object):
    drinkchoice = 0
    
    states = [
               State(name='initial'),
               State(name='calibration', on_enter='enter_calibration', on_exit="exit_calibration"),
               State(name='idle', on_enter='enter_idle'),
               State(name='mix_drink', on_enter='enter_mix_drink'),
               State(name='error', on_enter='enter_error'),
               State(name='done')
             ]
    transitions = [
                    {'trigger': 'start', 'source': 'initial', 'dest': 'calibration'},
                    {'trigger': 'calibration_completed', 'source': 'calibration', 'dest': 'idle'},
                    {'trigger': 'drink_picked', 'source': 'idle', 'dest': 'mix_drink'},
                    {'trigger': 'drink_mixed', 'source': 'mix_drink', 'dest': 'idle'},
                    {'trigger': 'error_occurred', 'source': ['idle', 'mix_drink'], 'dest': 'error'},
                    {'trigger': 'error_cleared', 'source': 'error', 'dest': 'idle'},
                    {'trigger': 'stop', 'source': 'idle', 'dest': 'done'}
                  ]
    machine = None
    currentTask = None
    rosnode = None

    def update(self):
        if self.currentTask is not None:
            self.currentTask()
 
    def __init__(self, rosnode):
        self.rosnode = rosnode
        self.machine = Machine(model=self, states=self.states, transitions=self.transitions, initial='initial', queued=True)
    
    def enter_calibration(self):
        print "enter_calibration"
        t = Timer(5.0, lambda: self.calibration_completed())
        t.start()
        
    def exit_calibration(self):
        print "exit_calibration"
        
    def enter_idle(self):
        print "enter_idle"
        self.drink_picked()
        
    def enter_error(self):
        print "enter_error"
        
    def enter_mix_drink(self):
        print "enter_mix_drink"
        self.currentTask = self.rosnode.mix_drink
    
    def exit_mix_drink(self):
        self.currentTask = None
        
    def enter_done(self):
        print "enter_done"
