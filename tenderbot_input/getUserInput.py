import time
import rospy
from std_msgs.msg import Int32



def get_key_input():
    try:
        drinkInput = int(raw_input('Input: '))
    except ValueError:
        print "Ivalid input"
    return drinkInput

def ask_for_input():

    print "Which drink to you want me to mix? You have to following options: \n Press 1 for Screwdriver with cream \n Press 2 for Russian breakfast \n Press 3 for White russian \n Awaiting input key for chosen drink..."

    drinkInput = get_key_input()

    if drinkInput == 1:
        print "You chose Screwdriver with cream"

    elif drinkInput == 2:
        print "You chose Russian breakfast"

    elif drinkInput == 3:
        print "You chose White russian"

    pub = rospy.Publisher("Input/DrinkChoice", Int32, queue_size = 1)
    rospy.init_node("Script", anonymous=True);
    pub.publish(drinkInput)
    time.sleep(1);
    

if __name__ == "__main__":
    ask_for_input()