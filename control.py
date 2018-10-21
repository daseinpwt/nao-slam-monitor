import matplotlib.backends.tkagg as tkagg
import matplotlib.image as mpimg
from matplotlib.lines import Line2D
import matplotlib.colors
import numpy as np
import thread
import time
from random import randint, random
import cv2
from naomanager import Nao
from logger import timestamp

im_odo = mpimg.imread('robot_odometry.png')
last_landmark = None

LEN_QUEUE = 1000
head_traj = 0
tail_traj = 0
q_traj = [None] * LEN_QUEUE

def refresh_canvas(canvas_pack):
    ax, fig, fig_photo, fig_canvas_agg = canvas_pack

    fig.canvas.draw()
    tkagg.blit(fig_photo, fig_canvas_agg.get_renderer()._renderer, colormode=2)

def draw_robot():
    ax = canvas_pack_traj[0]

    if draw_robot.state_odo is not None:
        draw_robot.state_odo[0].remove()
        draw_robot.state_odo[1].remove()

    draw_robot.state_odo = (
        ax.imshow(im_odo, extent=[pos[0]-16, pos[0]+16, pos[1]-16, pos[1]+16]),
        ax.arrow(pos[0], pos[1], 15*np.cos(pos[2]), 15*np.sin(pos[2]), head_width=10, head_length=10, edgecolor='red')
    )

    refresh_canvas(canvas_pack_traj)

draw_robot.state_odo = None

def draw_line(pnt1, pnt2):
    ax = canvas_pack_traj[0]

    l = Line2D([pnt1[0], pnt2[0]], [pnt1[1], pnt2[1]], color='#ff5050') 
    ax.add_line(l)

    refresh_canvas(canvas_pack_traj)

def nop(event):
    pass

def select_all_text(event):
    c = event.widget.winfo_class()
    if c == "Text":
        event.widget.tag_add("sel", "1.0", "end")
    elif c == "Entry":
        event.widget.select_range(0, 'end')

def remove_focus(event):
    event.widget.focus()

def move_forward(event):
    logger.log('Move forward')

    x = 0.5
    nao.motion.setWalkTargetVelocity(x, 0, 0, frequency)

def move_backward(event):
    logger.log('Move backward')

    x = -0.5
    nao.motion.setWalkTargetVelocity(x, 0, 0, frequency)

def turn_left(event):
    logger.log('Turn left')

    theta = 0.25
    nao.motion.setWalkTargetVelocity(x, y, theta, 0.15)

def turn_right(event):
    logger.log('Turn right')

    theta = -0.25
    nao.motion.setWalkTargetVelocity(x, y, theta, 0.15)

def standup(event):
    logger.log("Standup")

    nao.posture.post.goToPosture("StandInit", 0.5)

def rest(event):
    logger.log("Going to rest")

    nao.motion.post.rest()

def stop(event):
    logger.log('Stop')

    x = 0
    nao.motion.setWalkTargetVelocity(x, 0, 0, frequency)

def recog_color(hsv):
    h, s, v = hsv
    h *= 360
    color = None

    if h < 100 and s > 0.3:
        color = 'yellow'

    if 190 < h and h < 230 and s > 0.3:
        color = 'blue'

    if 100 < h and h < 190 and s > 0.25:
        color = 'green'

    if color is None:
        color = 'white'

    return color

def calc_range(w, bearing):
    if bearing > 32.0:
        if w >= 15:
            w -= 2
        else:
            w -= 1
    elif bearing > 21:
        w -= 1

    A = -14.00487805 
    B = 439.2439024
    return A * w + B 

def calc_bearing(off, limit):
    THETA_MAX = 60.97 # in degree
    return THETA_MAX * off / limit

def display_realtime_img(image, time, new_pos):
    global last_landmark

    ax = canvas_pack_vstream[0]
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Grab the x and y size and make a copy of the image
    ysize = image.shape[0]
    xsize = image.shape[1]
    detect_image = np.copy(image)

    # 320 x 240
    left_bottom = [50, 239]
    left_top = [60, 120]
    right_bottom = [270, 239]
    right_top = [260, 120]

    # Perform a linear fit (y=Ax+B) to each of the three sides of the triangle
    # np.polyfit returns the coefficients [A, B] of the fit
    fit_left = np.polyfit((left_bottom[0], left_top[0]), (left_bottom[1], left_top[1]), 1)
    fit_right = np.polyfit((right_bottom[0], right_top[0]), (right_bottom[1], right_top[1]), 1)
    fit_bottom = np.polyfit((left_bottom[0], right_bottom[0]), (left_bottom[1], right_bottom[1]), 1)
    fit_top = np.polyfit((left_top[0], right_top[0]), (left_top[1], right_top[1]), 1)

    # Find the region inside the lines
    XX, YY = np.meshgrid(np.arange(0, xsize), np.arange(0, ysize))

    region_thresholds = (YY > (XX*fit_left[0] + fit_left[1])) & \
                        (YY > (XX*fit_right[0] + fit_right[1])) & \
                        (YY < (XX*fit_bottom[0] + fit_bottom[1])) & \
                        (YY > (XX*fit_top[0] + fit_top[1]))

    # Convert RGB to HSV
    hsv_image = matplotlib.colors.rgb_to_hsv(image / 255.0)
    
    pink_thresholds = (hsv_image[:,:,0]*360 >= 320) & (hsv_image[:,:,0]*360 <= 350) & \
                      (hsv_image[:,:,1] > 0.3) & \
                      (hsv_image[:,:,2] > 0.3) & (hsv_image[:,:,2] < 0.7)

    YY, XX = (pink_thresholds & region_thresholds).nonzero()
    points = np.array(zip(XX, YY))

    dis_select = np.full((image.shape[0], image.shape[1]), False)
    
    if len(points) >= 20:
        mx, my = points.mean(axis=0)

        def distance(x1, x2, y1, y2):
            return ((x1-x2)**2 + (y1-y2)**2) ** 0.5

        cnt = len(points)
        m_dis = 0.0
        for point in points:
            m_dis += distance(point[0], mx, point[1], my) / cnt

        for y in range(ysize):
            for x in range(xsize):
                dis_select[y][x] = distance(x, mx, y, my) < m_dis * 1.6
                
    detect_region = pink_thresholds & region_thresholds & dis_select
    YY, XX = detect_region.nonzero()
    
    find_landmark = False
    if len(YY) > 0: # Found landmark
        find_landmark = True

        pg_left, pg_right, pg_top, pg_bottom = min(XX), max(XX), min(YY), max(YY)
        size = (pg_bottom - pg_top) * (pg_right - pg_left)

        detect_border_x = [pg_left, pg_right, pg_right, pg_left, pg_left]
        detect_border_y = [pg_bottom, pg_bottom, pg_top, pg_top, pg_bottom]
        
        above_point = (pg_left + pg_right) / 2, pg_top - (pg_bottom - pg_top) / 2
        below_point = (pg_left + pg_right) / 2, pg_bottom + (pg_bottom - pg_top) / 2

        above_color = recog_color(hsv_image[above_point[1], above_point[0]]) if above_point[1] > 0 else 'none'
        below_color = recog_color(hsv_image[below_point[1], below_point[0]]) if below_point[1] < ysize else 'none'

        if above_color == 'blue':
            landmark = 0
        elif below_color == 'blue':
            landmark = 1
        elif above_color == 'green':
            landmark = 2
        elif below_color == 'green':
            landmark = 3
        elif above_color == 'yellow':
            landmark = 4
        elif below_color == 'yellow':
            landmark = 5
        else:
            # can not recognize the sign
            landmark = -1

        w_pink = pg_right - pg_left
        off_pink = (pg_left + pg_right) / 2 - xsize / 2

        z_bearing = calc_bearing(off_pink, xsize / 2)
        z_range = calc_range(w_pink, abs(z_bearing))
        z_bearing = 90 - z_bearing

        if w_pink < 8:
            # the landmark is too far away, it is ignored
            # to reduce the measurement error
            landmark = -1

    else:
        landmark = -1
        # print('no landmark detected.\n')

    if landmark != -1:
        pos_logger.log("[%s]%s,%s,%s,1,%s,%s,%s" % (time, new_pos[0], new_pos[1], new_pos[2], landmark, z_range, z_bearing))
    else:
        pos_logger.log("[%s]%s,%s,%s,0" % (time, new_pos[0], new_pos[1], new_pos[2]))

    if landmark != last_landmark:
            last_landmark = landmark
            if landmark != -1:
                logger.log("detected landmark %s" % landmark)
            else:
                logger.log("no landmark detectd.")

    scan_border_x = [left_bottom[0], right_bottom[0], right_top[0], left_top[0], left_bottom[0]]
    scan_border_y = [left_bottom[1], right_bottom[1], right_top[1], left_top[1], left_bottom[1]]

    # refresh frame
    if display_realtime_img.video_frame is not None:
        for g in display_realtime_img.video_frame:
            if g is not None:
                g.remove()

    display_realtime_img.video_frame = (
        ax.imshow(detect_image),
        ax.plot(detect_border_x, detect_border_y, '-', lw=2, color='black')[0] if find_landmark else None,
        ax.plot(scan_border_x, scan_border_y, 'b--', lw=2)[0]
    )
    refresh_canvas(canvas_pack_vstream)

display_realtime_img.video_frame = None

def capture(time, new_pos, display=True):
    img = None

    result = nao.video_device.getImageRemote(nao.capture_device)
    if result == None:
        logger.log('cannot capture.')
    elif result[6] == None:
        logger.log('no image data string.')
    else: 
        values = map(ord, list(result[6]))
        img = np.reshape(values, (240, 320, 3)).astype('uint8')

    if display:
        thread.start_new_thread(display_realtime_img, (img, time, new_pos))

    return img

def get_pose():
    _pos = nao.motion.getRobotPosition(False)
    return (_pos[0] * 100, _pos[1] * 100, _pos[2])

def update_traj():
    global pos
    global q_traj, head_traj, tail_traj

    while True:
        while head_traj != tail_traj:
            head_traj = (head_traj + 1) % LEN_QUEUE
            new_pos = q_traj[head_traj]
            draw_line((new_pos[0], new_pos[1]), (pos[0], pos[1]))
            pos = new_pos
            draw_robot()
        time.sleep(0) # yield

def position_fetcher():
    global pos
    global q_traj, head_traj, tail_traj

    while True:
        new_time = timestamp()
        new_pos = get_pose()

        capture(new_time, new_pos)

        next_p = (tail_traj + 1) % LEN_QUEUE
        q_traj[next_p] = new_pos
        tail_traj = next_p

        time.sleep(0.2)

def mark_point(event):
    pos_logger.log("[%s]mark" % timestamp())

def elegant_exit(event):
    print('elegant exit')

    pos_logger.close()
    nao.unsubscribe_camera()
    mw.destroy()

def setup_window_control(
        main_window, 
        _logger, _pos_logger,
        _canvas_pack_traj,
        _canvas_pack_vstream,
        nao_ip, nao_port):

    global mw
    global logger, pos_logger
    global canvas_pack_traj, canvas_pack_vstream
    global nao, pos, x, y, theta, frequency, CommandFreq

    canvas_pack_traj = _canvas_pack_traj
    canvas_pack_vstream = _canvas_pack_vstream

    mw = main_window
    logger = _logger
    pos_logger = _pos_logger
    
    mw.bind("<Button-1>", remove_focus)

    # Mac
    mw.bind('<Command-a>', nop)
    
    # Linux
    # mw.bind('<Control-a>', nop)
    
    mw.bind('w', move_forward)
    mw.bind('s', move_backward)
    mw.bind('a', turn_left)
    mw.bind('d', turn_right)
    mw.bind('q', rest)
    mw.bind('e', standup)
    mw.bind('x', stop)
    mw.bind('m', mark_point)

    mw.bind('<Control-q>', elegant_exit)

    thread.start_new_thread(data_fetcher, ())

    # ============ Connect to Robot ============
    logger.log('connecting to the robot...')
    nao = Nao(nao_ip, nao_port)

    x = 0.0
    y = 0.0
    theta = 0.0
    frequency = 0.3
    CommandFreq = 0.5

    nao.motion.wakeUp()
    nao.posture.goToPosture("StandInit", 0.5)

    time.sleep(1.0)

    nao.motion.setStiffnesses("Head", 1.0)
    names  = ["HeadYaw", "HeadPitch"]
    angles  = [0.0, -0.12]
    nao.motion.setAngles(names, angles, 0.2)

    time.sleep(1.0)

    logger.log('Ready to go!!!')
    pos = get_pose()
    logger.log("Init pos: %s" % str(pos))
    draw_robot()

    thread.start_new_thread(position_fetcher, ())
    thread.start_new_thread(update_traj, ())
