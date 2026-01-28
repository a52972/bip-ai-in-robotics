#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import speech_recognition as sr
import os
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ctypes import *
from contextlib import contextmanager

# --- CONFIGURATION ---
ROBOT_NAME = "Hospital Guide"
WAKE_WORDS = ["hospital", "robot", "assistant", "guide", "assistance"]

LOCATIONS = {
    "CAFETERIA":    {"x": 2.4,  "y": -0.7,  "w": 1.0},
    "RECEPTION":    {"x": 0.0, "y": 0.0,  "w": 1.0},
    "WAITING ROOM": {"x": 1.5,  "y": 0.8,  "w": 1.0} 
}

# --- ALSA SILENCER ---
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextmanager
def no_alsa_error():
    try:
        asound = cdll.LoadLibrary('libasound.so')
        asound.snd_lib_error_set_handler(c_error_handler)
        yield
        asound.snd_lib_error_set_handler(None)
    except:
        yield

# --- AUTO-DETECT MICROPHONE ---
def find_usb_mic():
    """
    Scans for a microphone with 'USB' in its name.
    Returns the index if found, otherwise returns None (Default).
    """
    rospy.loginfo("Scanning for USB Microphone...")
    mic_list = sr.Microphone.list_microphone_names()
    
    for index, name in enumerate(mic_list):
        # Check for common USB mic names
        if "USB" in name or "UAC" in name or "PnP" in name:
            rospy.loginfo("Auto-detected USB Mic at Index {}: {}".format(index, name))
            return index
            
    rospy.logwarn("No USB Mic found! Using default system mic.")
    return None # Lets the system decide

# --- TEXT TO SPEECH ---
def speak(text):
    rospy.loginfo("ROBOT SAYS: " + text)
    safe_text = text.replace("'", "")
    cmd = "espeak -v en-us+f3 -s 150 '{}' --stdout | aplay -D plughw:2,0 >/dev/null 2>&1".format(safe_text)
    os.system(cmd)

# --- NAVIGATION CLIENT ---
class Navigator:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server() 
        rospy.loginfo("Connected to Navigation Server!")

    def goto(self, location_name):
        target = LOCATIONS.get(location_name)
        if not target:
            return
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target['x']
        goal.target_pose.pose.position.y = target['y']
        goal.target_pose.pose.orientation.w = target['w']

        self.client.send_goal(goal)
        self.client.wait_for_result()
        speak("I have arrived at the " + location_name.lower())

    def stop(self):
        self.client.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())

# --- VOICE LOGIC ---
def get_voice_command(recognizer, mic):
    # REMOVED SILENCER HERE for debugging
    try:
        with mic as source:
            print("\n" + "="*40)
            print("LISTENING... (Say: '{} go to reception')".format(ROBOT_NAME))
            print("="*40)
            
            # Removed adjust_for_ambient_noise to prevent freezing
            audio = recognizer.listen(source, timeout=5, phrase_time_limit=8)
            print("Processing...")
            
            text = recognizer.recognize_google(audio)
            return text.lower()

    except sr.UnknownValueError:
        return None
    except sr.RequestError:
        speak("I cannot connect to Google.")
        return None
    except sr.WaitTimeoutError:
        return None
    except Exception as e:
        print("Error: {}".format(e))
        return None

def main():
    rospy.init_node('voice_commander_google')
    
    speak("System starting up.")
    nav = Navigator()
    r = sr.Recognizer()
    
    # --- AUTO DETECT LOGIC ---
    # We silence errors during detection scan to keep logs clean
    with no_alsa_error():
        usb_index = find_usb_mic()
        mic = sr.Microphone(device_index=usb_index)

    speak("I am ready. Where to?")

    while not rospy.is_shutdown():
        command = get_voice_command(r, mic)

        if command:
            print("HEARD: '{}'".format(command))
            
            if not any(w in command for w in WAKE_WORDS):
                print("Ignored (No wake word)")
                continue

            if "cafeteria" in command or "coffee" in command:
                speak("Going to the cafeteria.")
                nav.goto("CAFETERIA")
            elif "reception" in command or "information" in command:
                speak("Heading to reception.")
                nav.goto("RECEPTION")
            elif "waiting" in command: 
                speak("Taking you to the waiting room.")
                nav.goto("WAITING ROOM")
            elif "stop" in command:
                speak("Stopping.")
                nav.stop()
            else:
                speak("I heard you, but I do not know that place.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass