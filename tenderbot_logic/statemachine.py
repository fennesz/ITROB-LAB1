import roshelper
from std_msgs.msg import String
from threading import Timer
from transitions import Machine, State, Transition

# github.com/pytransitions/transitions

node_input = "Input"
node_vision = "Vision"
node_arm = "Arm"

i = roshelper.Node(node_input, anonymous=False)
v = roshelper.Node(node_vision, anonymous=False)
a = roshelper.Node(node_arm, anonymous=False)

class TenderBotStateMachine(object):
    drinkchoice = 0
    
    states = [
               State(name='initial'),
               State(name='setup', on_enter='enter_setup'),
               State(name='calibration', on_enter='enter_calibration', on_exit="exit_calibration"),
               State(name='idle', on_enter='enter_idle'),
               State(name='mix_drink', on_enter='enter_mix_drink'),
               State(name='error', on_enter='enter_error'),
               State(name='done')
             ]
    transitions = [
                    {'trigger': 'start', 'source': 'initial', 'dest': 'setup'},
                    {'trigger': 'setup_completed', 'source': 'setup', 'dest': 'calibration'}, 
                    {'trigger': 'calibration_completed', 'source': 'calibration', 'dest': 'idle'},
                    {'trigger': 'drink_picked', 'source': 'idle', 'dest': 'mix_drink'},
                    {'trigger': 'drink_mixed', 'source': 'mix_drink', 'dest': 'idle'},
                    {'trigger': 'error_occurred', 'source': ['idle', 'mix_drink'], 'dest': 'error'},
                    {'trigger': 'error_cleared', 'source': 'error', 'dest': 'idle'},
                    {'trigger': 'stop', 'source': 'idle', 'dest': 'done'}
                  ]
    machine = None
    currentTask = None

    @i.subscriber("/input", String)
    def listener(word):
        rospy.loginfo(rospy.get_caller_id() + " : I heard %s" % word.data)

    def update(self):
        if self.currentTask is not None:
            self.currentTask()
 
    def __init__(self):
        self.machine = Machine(model=self, states=self.states, transitions=self.transitions, initial='initial', queued=True)
    
    def enter_initial(self):
        self.start()

    def enter_setup(self):        
        self.currentTask = None
        print "enter_setup"
        #Subscribe to input topic        
        #Subscribe to shape
        #Subscribe to arm
        self.setup_completed()

    def enter_calibration(self):
        self.currentTask = None
        #Perform calibration routine
        print "enter_calibration"
        self.calibration_completed()
        
    def exit_calibration(self):
        print "exit_calibration"
        
    def enter_idle(self):
        self.currentTask = None
        print "enter_idle"
        
    def enter_error(self):
        self.currentTask = None
        print "enter_error"
        
    def enter_mix_drink(self):
        self.currentTask = None
        print "enter_mix_drink"
        
    def enter_done(self):
        self.currentTask = None
        print "enter_done"
