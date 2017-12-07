import time
import pygame

def get_key_input():
    try:
        drinkInput = int(raw_input('Input'))
    except ValueError:
        print "Ivalid input"
    return drinkInput

def ask_for_input(self, drinkInput):

    print "Which drink to you want me to mix? You have to following options:"
    print "Press 1 for Screwdriver with cream"
    time.wait(5)
    print "Press 2 for Russian breakfast"
    time.wait(5)
    print "Press 3 for White russian"
    time.wait(5)
    print "Awaiting input key for chosen drink..."
    get_key_input()
    if drinkInput == 1:
        print "You chose Screwdriver with cream"
        #one_pressed()

    elif drinkInput == 2:
        print "You chose Russian breakfast"
        #two_pressed()

    elif drinkInput == 3:
        print "You chose White russian"
        #three_pressed()

    else:
        print "Please choose your drink"
        print "Awaiting input key for chosen drink..."


#def one_pressed():

#def two_pressed():

#def three_pressed():