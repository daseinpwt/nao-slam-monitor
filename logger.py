from Tkinter import END
import datetime 

def timestamp():
    curr_time = datetime.datetime.now()
    return curr_time.strftime('%H:%M:%S.%f')

class Logger(object):
    def __init__(self, text_area):
        self.text_area = text_area

    def log(self, msg):
        msg = "[%s] %s" % (timestamp(), msg)

        self.text_area.configure(state='normal')
        self.text_area.insert(END, msg + '\n')
        self.text_area.configure(state='disabled')
        self.text_area.yview(END)

        print(msg)

class FileLogger(object):
    def __init__(self, text_area, filename):
        self.text_area = text_area
        self.file = open(filename, 'w')

    def log(self, msg):
        self.text_area.configure(state='normal')
        self.text_area.insert(END, msg + '\n')
        self.text_area.configure(state='disabled')
        self.text_area.yview(END)

        self.file.write(msg + '\n')

    def close(self):
        self.file.close()


