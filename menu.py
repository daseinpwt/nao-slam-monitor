from Tkinter import Menu
from tkMessageBox import *

def show_keyboard_control_dialog():
    title = 'Keyboard Control'
    content = (
        'w: move forward\n'
        's: move backward\n'
        'a: turn left\n'
        'd: turn right\n'
        'q: rest\n'
        'e: standup\n'
        'x: stop\n'
        'm: mark\n'
        'Control-q: quit\n'
    )

    showinfo(title, content)

def show_about_dialog():
    title = 'Nao SLAM Monitor v0.1'
    content = (
        'This tool is developed for controlling the Nao Robot and performing'
        ' the landmark detection at the same.\n'
        '\n'
        'Author: Wentao Pan [daseinpwt@gmail.com]'
    )

    showinfo(title, content)

def setup_menu(main_window):
    global mw
    mw = main_window

    menubar = Menu(mw)

    helpmenu = Menu(menubar, tearoff=0)
    helpmenu.add_command(label="Keyboard Control", command=show_keyboard_control_dialog)
    helpmenu.add_command(label="About", command=show_about_dialog)
    menubar.add_cascade(label="Help", menu=helpmenu)

    mw.config(menu=menubar)

