import PIL.Image
from tkinter import *
import tkFont
from PIL import ImageTk, Image
from threading import Thread
import cv2
import time
import os
from rospkg import RosPack


class GUI(Tk):
    # *args, **kwargs
    def __init__(self, session):
        Tk.__init__(self)

        # the container is where we'll stack a bunch of frames
        # on top of each other, then the one we want visible
        # will be raised above the others
        self.geometry('1000x680')
        container = Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        for F in (VideoGUI, ConvGUI):
            page_name = F.__name__
            frame = F(container, self, session)
            self.frames[page_name] = frame

            # put all of the pages in the same location;
            # the one on the top of the stacking order
            # will be the one that is visible.
            frame.grid(row=0, column=0, sticky="nsew")

        self.run_gui(["VideoGUI"])

    def run_gui(self, page_name):
        """
        '''Show a frame for the given page name'''
        frame = self.frames[page_name]
        frame.tkraise()
        """

        '''Show a frame for the given page name'''
        for frame in self.frames.values():
            frame.grid_remove()
        for f in page_name:
            frame = self.frames[f]
            frame.grid(column=frame.col)


class ConvGUI(Frame):
    def __init__(self, parent, controller, session):
        Frame.__init__(self, parent, bg="black")

        self.controller = controller
        self.side = RIGHT
        self.col = 1

        self.const_notify_lbl = Label(self, text=" ", bg="black", fg="white",
                                      pady=20, font=tkFont.Font(family="Arial", size=14, weight="bold"),
                                      wraplength=600, justify="left")
        self.const_notify_lbl.grid(column=2, row=1, sticky=W)

        self.ques1_lbl = Label(self, text=" ", bg="black", fg="#40E0D0",
                               pady=10, font=tkFont.Font(family="Times", size=14, weight="bold"),
                               wraplength=600, justify="left")
        self.ques1_lbl.grid(column=2, row=2, sticky=W)

        self.ques2_lbl = Label(self, text=" ", bg="black", fg="#40E0D0",
                               pady=10, font=tkFont.Font(family="Times", size=14, weight="bold"),
                               wraplength=600, justify="left")
        self.ques2_lbl.grid(column=2, row=3, sticky=W)

        self.ques3_lbl = Label(self, text=" ", bg="black", fg="#40E0D0",
                               pady=10, font=tkFont.Font(family="Times", size=14, weight="bold"),
                               wraplength=600, justify="left")
        self.ques3_lbl.grid(column=2, row=4, sticky=W)


class VideoGet:
    """Class that continuously gets frames from a VideoCapture object
    with a dedicated thread"""

    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.get, args=()).start()
        return self

    def get(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()

    def stop(self):
        self.stopped = True


class VideoGUI(Frame):
    def __init__(self, parent, controller, session):
        Frame.__init__(self, parent)
        self.controller = controller
        self.lmain = Label(self)
        self.session = session
        self.lmain.pack(side="top", fill="x")
        self.side = LEFT
        self.col = 0
        self.counter = 0

        self.image_path = os.path.join(RosPack().get_path("visu_ros_msg"), "include", "visu_ros_msg")
        if self.session == "robot":
            self.video_getter = VideoGet(0).start()
        self.s_time = time.time()
        self.end_time = 0
        self.video_stream()

    def video_stream(self):
        img = None
        if self.session == "tts":
            img = PIL.Image.open(os.path.join(self.image_path, "tts.png"))
            img = img.resize((self.winfo_width(), self.winfo_height()))
        else:
            try:
                frame = self.video_getter.frame
                cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
                img = Image.fromarray(cv2image)
            except:
                pass
        if img is not None:
            imgtk = ImageTk.PhotoImage(image=img)
            self.lmain.imgtk = imgtk
            self.lmain.configure(image=imgtk)

        self.lmain.after(1, self.video_stream)
