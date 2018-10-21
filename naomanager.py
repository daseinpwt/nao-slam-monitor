from collections import defaultdict
import time
import os
import datetime
import sys

from naoqi import ALProxy

DEFAULT_PORT = 9559

class Nao(object):
    """ Represents a Nao. Initializes the proxies we want.
        Additionally provides methods for logging the angles."""
    def __init__(self, ip, port=9559, logfolder=None):
        super(Nao, self).__init__()

        self.ip = ip
        self.port = port

        print(self.ip, self.port)

        self.motion = ALProxy('ALMotion', ip, port)
        self.behavior = None
        self.posture = ALProxy('ALRobotPosture', ip ,port)

        cam_proxy = ALProxy("ALVideoDevice", ip, port)
        AL_kTopCamera = 0 # 0: topcamera, 1: bottomcamera
        AL_kQVGA = 1      # 320x240
        AL_kBGRColorSpace = 13
        self.video_device, self.capture_device = cam_proxy, cam_proxy.subscribeCamera("top", AL_kTopCamera, AL_kQVGA, AL_kBGRColorSpace, 10)
     
        self.jointnames = self.motion.getJointNames("Body")
        self.timestamps = []
        self.desiredangles = defaultdict( lambda : list() )
        self.realangles = defaultdict( lambda : list() )

        self.initstiffness()

        # Create logging folder if needed
        self.logfolder = logfolder
        self.logfp = None
        if logfolder:
            if not os.path.exists(logfolder):
                os.mkdir(logfolder)

    def initstiffness(self):
        self.motion.stiffnessInterpolation('Body', 0.5, 1.0)

    def releasestiffness(self):
        self.motion.stiffnessInterpolation('Body', 0.0, 1.0)

    def startlogging(self):
        if not self.logfolder:
            return
        now = datetime.datetime.now()
        self.logname = '%s_%s-%.4d%.2d%.2d%.2d%.2d%.2d' % (self.ip, self.port, now.year, now.month, now.day, now.hour, now.minute, now.second)
        self.logfp = open(os.path.join(self.logfolder, self.logname), 'wb')

        # First line of the log file contains the joint names
        self.logfp.write('timestamp')
        for n in self.jointnames:
            self.logfp.write('\t%s\t ' % (n))
        self.logfp.write('\n')

    def stoplogging(self):
        if self.logfp:
            self.logfp.close()
            self.logfp = None

    def unsubscribe_camera(self):
        self.video_device.unsubscribe("top")

    def stop(self):
        print('in nao stop')
        self.motion.rest()
        self.motion.killAll()

        self.releasestiffness()
        self.motion = None
        self.behavior = None
        self.posture = None
        self.video_device.unsubscribe("top")
        self.video_device = None
        self.stoplogging()

    def updateanglehistory(self):
        self.timestamps.append(time.time())
        if self.logfolder:
            self.logfp.write('%f' % (self.timestamps[-1]))

        n = len(self.timestamps)
        self.timestamps = self.timestamps[max(0,n-100):n]

        angles = self.motion.getAngles('Body', False)
        anglesreal = self.motion.getAngles('Body', True)
        for i, name in enumerate(self.jointnames):
            self.desiredangles[name].append(angles[i])
            self.realangles[name].append(anglesreal[i])

            n = len(self.desiredangles[name])
            self.desiredangles[name] = self.desiredangles[name][max(0,n-100):n]
            n = len(self.realangles[name])
            self.realangles[name] = self.realangles[name][max(0,n-100):n]

            if self.logfolder:
                self.logfp.write('\t%f\t%f' % (angles[i], anglesreal[i]))
        if self.logfolder: self.logfp.write('\n')

    def gettimestamps(self): return self.timestamps

class NaoCallAll(object):
    """ Wraps a call to multiple Naos/attributes.
        Returns a new call wrapper in case you try to access an attribute."""
    def __init__(self, objects, attr):
        super(NaoCallAll, self).__init__()

        self.attrobjects = []

        for o in objects:
            self.attrobjects.append( o.__getattribute__(attr) )

    def __getattr__(self, attr):

        return NaoCallAll(self.attrobjects, attr)

    def __call__(self, *args, **kwargs):
        results = []
        for a in self.attrobjects:
            results.append( a(*args, **kwargs) )
        return results

    def __getitem__(self, k):
        results = []
        for a in self.attrobjects:
            results.append( a.__getitem__(k) )
        return results


class NaoManager(list):
    """ Manages a list of nao's.
        Works like a normal list, except a call on the
        list will result in the method being called on all nao's in the list."""
    def __init__(self, *args, **kwargs):
        super(NaoManager, self).__init__(*args, **kwargs)

    def __getattr__(self, attr):
        return NaoCallAll(self, attr)

    def addnao(self, ip, port=9559, logfolder=None):
        self.append( Nao(ip, port, logfolder) )