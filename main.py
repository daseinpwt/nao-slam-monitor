from Tkinter import *
from ScrolledText import ScrolledText
from PIL import Image, ImageTk
from control import setup_window_control, select_all_text
from menu import setup_menu
from logger import Logger, FileLogger
import numpy as np
import matplotlib as mpl
import matplotlib.backends.tkagg as tkagg
from matplotlib.backends.backend_agg import FigureCanvasAgg
import matplotlib.image as image
import time
from naoqi import ALProxy

NAO_IP = 'sam.local'
NAO_PORT = 9559
LOG_FILE = 'data.log'

def draw_figure(canvas, figure, loc=(0, 0)):
    """ Draw a matplotlib figure onto a Tk canvas

    loc: location of top-left corner of figure on canvas in pixels.
    Inspired by matplotlib source: lib/matplotlib/backends/backend_tkagg.py
    """
    figure_canvas_agg = FigureCanvasAgg(figure)
    figure_canvas_agg.draw()

    figure_x, figure_y, figure_w, figure_h = figure.bbox.bounds
    figure_w, figure_h = int(figure_w), int(figure_h)

    photo = PhotoImage(master=canvas, width=figure_w, height=figure_h)

    # Position: convert from top-left anchor to center anchor
    canvas.create_image(loc[0] + figure_w/2, loc[1] + figure_h/2, image=photo)

    # Unfortunately, there's no accessor for the pointer to the native renderer
    tkagg.blit(photo, figure_canvas_agg.get_renderer()._renderer, colormode=2)

    # Return a handle which contains a reference to the photo object
    # which must be kept live or else the picture disappears
    return figure_canvas_agg, photo


mw = Tk()
mw.title('Nao SLAM Monitor')
mw.geometry('1160x900')

# ======== Realtime Video Streaming Frame ========
frm_vstream = Frame(mw, width=480, height=360, bg='red')
frm_vstream.place(x=40, y=25, width=480, height=360)

canvas_vstream = Canvas(frm_vstream, width=480, height=360)
canvas_vstream.pack()

fig_vstream = mpl.figure.Figure(figsize=(4.8, 3.6))
ax_vstream = fig_vstream.add_subplot(111, aspect='equal')

fig_canvas_agg_vstream, fig_photo_vstream = draw_figure(canvas_vstream, fig_vstream, loc=(0, 0))

# ======== Estimated and Odometric trajactories Frame ========
frm_traj = Frame(mw, width=660, height=460, bg='red')
frm_traj.place(x=520, y=-25, width=660, height=460)

canvas_traj = Canvas(frm_traj, width=660, height=460)
canvas_traj.pack()

# Generate some example data
data_X = np.linspace(0, 2 * np.pi, 50)
data_Y = np.sin(data_X)

# Create the figure we desire to add to an existing canvas
fig_traj = mpl.figure.Figure(figsize=(6.6, 4.6))
ax_traj = fig_traj.add_subplot(111, aspect='equal', facecolor='#008000')

ax_traj.set_xlim(-330, 330)
ax_traj.set_ylim(-230, 230)

# The marks are placed in an eight-shape figure on 
# locations (0,0), (100,0), (100,-100), (-150,0) and (-150,100) cm
markers = np.array([[ 0, 100,  100, -150, -150],
                    [ 0,   0, -100,    0,  100]])
n_markers = 5

for i in range(n_markers):
    ax_traj.plot(markers[0, i], markers[1, i], 'x', color='black', markersize=5)

landmarks = np.array([[ -315, -315,    0,   0,  315, 315],
                      [ -215,  215, -215, 215, -215, 215]])
landmark_colors = [['blue',    'magenta',  '#003300',   'magenta', '#ffcc00', 'magenta'],
                   ['magenta',    'blue',  'magenta',  '#003300', 'magenta', '#ffcc00']]
n_landmarks = 6

for i in range(n_landmarks):
    ax_traj.plot(landmarks[0, i], landmarks[1, i], 'o', color=landmark_colors[1][i], markersize=8)
    ax_traj.plot(landmarks[0, i], landmarks[1, i], 'o', color=landmark_colors[0][i], markersize=4)

# Keep this handle alive, or else figure will disappear
fig_canvas_agg_traj, fig_photo_traj = draw_figure(canvas_traj, fig_traj, loc=(0, 0))

# ======== User Control Logging Frame ========
frm_log = Frame(mw, width=480, height=330, highlightbackground='gray', highlightcolor='black', highlightthickness=2)
frm_log.place(x=40, y=430, width=480, height=330)

log_text_area = ScrolledText(
    master=frm_log,
    # wrap=mw.WORD,
    highlightthickness=0
)
log_text_area.configure(state='disabled')
log_text_area.pack(fill=BOTH)

log_text_area.bind('<Command-a>', select_all_text)
# log_text_area.bind('<Control-a>', select_all_text)

# ======== Odometry and Measurement Logging Frame ========
frm_pos = Frame(mw, width=511, height=330, highlightbackground='gray', highlightcolor='black', highlightthickness=2)
frm_pos.place(x=603, y=430, width=511, height=330)

pos_text_area = ScrolledText(
    master=frm_pos,
    # wrap=mw.WORD,
    highlightthickness=0
)
pos_text_area.configure(state='disabled')
pos_text_area.pack(fill=BOTH)

pos_text_area.bind('<Command-a>', select_all_text)

setup_window_control(
    mw,
    Logger(log_text_area), FileLogger(pos_text_area, LOG_FILE),
    (ax_traj, fig_traj, fig_photo_traj, fig_canvas_agg_traj),
    (ax_vstream, fig_vstream, fig_photo_vstream, fig_canvas_agg_vstream),
    NAO_IP, NAO_PORT
)
setup_menu(mw)
mw.mainloop()

