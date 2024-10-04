#python

import tkinter as tk
from tkinter import ttk

combo1_value = ''
combo2_value = ''


def startSimulation():
    global combo1_value, combo2_value
    if combo1_value != combo2_value:
        sim.setStringSignal("startPoint", combo1_value)
        sim.setStringSignal("stopPoint", combo2_value)
        sim.setStringSignal("startSimulation", "True")
        print(">> Simulation started")
        print(f">> [StartPoint: {combo1_value} | StopPoint: {combo2_value}]")
    else:
        print(">> [!] You cannot choose same start and stop points")
    

def enableWall():
    sim.setStringSignal("enableRandomWall", "True")
    print(">> Random wall inserted")


def update_combo1_value(*args):
    global combo1_value
    combo1_value = combo1_var.get()


def update_combo2_value(*args):
    global combo2_value
    combo2_value = combo2_var.get()


def loadInterface():
    global combo1_var, combo2_var  
    root = tk.Tk()
    root.title("Self-Driving Tool")
    root.geometry("400x260")

    label1 = tk.Label(root, text="Start Point")
    label1.pack(pady=5)

    combo1_var = tk.StringVar()
    combo1 = ttk.Combobox(root, textvariable=combo1_var, values=[chr(i) for i in range(ord('A'), ord('P') + 1)])
    combo1.pack(pady=5)
    combo1_var.trace('w', update_combo1_value)

    label2 = tk.Label(root, text="Stop Point")
    label2.pack(pady=5)
    combo2_var = tk.StringVar()
    combo2 = ttk.Combobox(root, textvariable=combo2_var, values=[chr(i) for i in range(ord('A'), ord('P') + 1)])
    combo2.pack(pady=5)
    combo2_var.trace('w', update_combo2_value)

    button1 = tk.Button(root, text="Start Simulation", command=startSimulation)
    button1.pack(pady=20)
    button2 = tk.Button(root, text="Enable Wall", command=enableWall)
    button2.pack(pady=20)

    root.mainloop()


def sysCall_init():
    sim = require('sim')
    sim.setStringSignal("startSimulation", "False")
    sim.setStringSignal("enableRandomWall", "False")


def sysCall_thread():
    loadInterface()
