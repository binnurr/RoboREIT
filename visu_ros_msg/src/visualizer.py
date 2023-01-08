#!/usr/bin/env python
import rospy
from visu_ros_msg.gui import GUI
from std_msgs.msg import String, Bool, Int16
from fsc_py_planner.msg import GUItext
import time
import sys


class Visualizer:
    def __init__(self, session_type):
        args = rospy.myargv(argv=sys.argv)
        session_mode = args[1]
        self.gui = GUI(session_mode)
        rospy.init_node('visualizer_node')
        topic_name = 'promptQues'
        self.subscriber = rospy.Subscriber(topic_name, GUItext, self.callback, queue_size=1)
        self.subscriber2 = rospy.Subscriber("selected_ques", String, self.callback2, queue_size=1)
        self.subscriber3 = rospy.Subscriber("selected_incorr_ques", String, self.callback3, queue_size=1)
        self.subscriber4 = rospy.Subscriber("repeat_request", String, self.callback4, queue_size=1)
        self.subscriber5 = rospy.Subscriber("feedback_eval_state", Int16, self.callback5, queue_size=1)
        self.subscriber6 = rospy.Subscriber("selected_feedback_ques", String, self.callback6, queue_size=1)
        self.subscriber7 = rospy.Subscriber("exp_finished", String, self.callback7, queue_size=1)
        self.subscriber8 = rospy.Subscriber('promptQues_Feedback', GUItext, self.callback8, queue_size=1)
        self.subscriber9 = rospy.Subscriber('feedback_stat', String, self.callback9, queue_size=1)


        self.publisher = rospy.Publisher("feedback_exp_finished", Int16, queue_size=1)
        self.selected_ques = -1
        self.selected_incorrect_ques = -1

    def callback(self, msg):
        self.configure_label(msg)

    def callback8(self, msg):
        self.configure_label_feedback(msg)

    def callback2(self, msg):
        self.selected_ques = int(msg.data)
        self.highlight_selected_ques(self.selected_ques)

    def callback3(self, msg):
        self.selected_incorrect_ques = int(msg.data)
        self.highlight_selected_incorrect_ques(self.selected_incorrect_ques)

    def callback4(self, msg):
        self.gui.run_gui(["VideoGUI"])

    def callback5(self, msg):
        self.highlight_feedback_eval(msg.data)

    def callback6(self, msg):
        self.selected_ques = int(msg.data)
        self.highlight_selected_feedback_ques(self.selected_ques)

    def callback7(self, msg):
        pass
        #self.gui.run_gui(["VideoGUI"])

    def callback9(self, msg):
        self.gui.frames["ConvGUI"].const_notify_lbl.configure(text=msg.data)
        self.gui.frames["ConvGUI"].ques1_lbl.configure(text="", bg="black")
        self.gui.frames["ConvGUI"].ques2_lbl.configure(text="", bg="black")
        self.gui.frames["ConvGUI"].ques3_lbl.configure(text="", bg="black")
        self.gui.run_gui(["VideoGUI", "ConvGUI"])

    def configure_label(self, msg):
        """
        if msg.prev_answer != "":
            self.gui.frames["ConvGUI"].prev_answer_lbl.configure(text="Customer responded:\n" + msg.prev_answer)
        """
        self.gui.frames["ConvGUI"].const_notify_lbl.configure(text="Select your response from below and say loudly to "
                                                                   "the client.\n "
                                                                   "If you want to listen client's response again say "
                                                                   "\"Please repeat\".")
        self.gui.frames["ConvGUI"].ques1_lbl.configure(text=msg.ques1, bg="black")
        self.gui.frames["ConvGUI"].ques2_lbl.configure(text=msg.ques2, bg="black")
        self.gui.frames["ConvGUI"].ques3_lbl.configure(text=msg.ques3, bg="black")
        self.gui.run_gui(["VideoGUI", "ConvGUI"])

    def configure_label_feedback(self, msg):
        pre_text = ""
        if msg.prev_answer != "":
            pre_text = "Mary had responded:\n" + msg.prev_answer + "\n"

        self.gui.frames["ConvGUI"].const_notify_lbl.configure(text=pre_text+"Select your response from below and say loudly to "
                                                                   "the client.\n ")
        self.gui.frames["ConvGUI"].ques1_lbl.configure(text=msg.ques1, bg="black")
        self.gui.frames["ConvGUI"].ques2_lbl.configure(text=msg.ques2, bg="black")
        self.gui.frames["ConvGUI"].ques3_lbl.configure(text=msg.ques3, bg="black")
        self.gui.run_gui(["VideoGUI", "ConvGUI"])

    def highlight_selected_ques(self, selected_ques):
        if selected_ques == 1:
            self.gui.frames["ConvGUI"].ques1_lbl.configure(bg="yellow")
        elif selected_ques == 2:
            self.gui.frames["ConvGUI"].ques2_lbl.configure(bg="yellow")
        elif selected_ques == 3:
            self.gui.frames["ConvGUI"].ques3_lbl.configure(bg="yellow")
        time.sleep(1)
        self.gui.run_gui(["VideoGUI"])

    def highlight_selected_feedback_ques(self, selected_ques):
        if selected_ques == 1:
            self.gui.frames["ConvGUI"].ques1_lbl.configure(bg="yellow")
        elif selected_ques == 2:
            self.gui.frames["ConvGUI"].ques2_lbl.configure(bg="yellow")
        elif selected_ques == 3:
            self.gui.frames["ConvGUI"].ques3_lbl.configure(bg="yellow")

    def highlight_selected_incorrect_ques(self, selected_ques):
        print("I am redding", selected_ques)
        if selected_ques == 1:
            self.gui.frames["ConvGUI"].ques1_lbl.configure(bg="red")
        elif selected_ques == 2:
            self.gui.frames["ConvGUI"].ques2_lbl.configure(bg="red")
        elif selected_ques == 3:
            self.gui.frames["ConvGUI"].ques3_lbl.configure(bg="red")

    def highlight_feedback_eval(self, feedback_eval_state):
        if feedback_eval_state == 1:
            h_color = "green"
        else:
            h_color = "red"
        if self.selected_ques == 1:
            self.gui.frames["ConvGUI"].ques1_lbl.configure(bg=h_color)
        elif self.selected_ques == 2:
            self.gui.frames["ConvGUI"].ques2_lbl.configure(bg=h_color)
        elif self.selected_ques == 3:
            self.gui.frames["ConvGUI"].ques3_lbl.configure(bg=h_color)

        if feedback_eval_state != 1:
            time.sleep(2)
            all_answers = [1, 2, 3]
            all_answers.remove(self.selected_ques)
            all_answers.remove(self.selected_incorrect_ques)
            correct_answer = all_answers[0]
            if feedback_eval_state == -1:
                h_color = "red"
            else:
                h_color = "green"
            if correct_answer == 1:
                self.gui.frames["ConvGUI"].ques1_lbl.configure(bg=h_color)
            elif correct_answer == 2:
                self.gui.frames["ConvGUI"].ques2_lbl.configure(bg=h_color)
            elif correct_answer == 3:
                self.gui.frames["ConvGUI"].ques3_lbl.configure(bg=h_color)

        if feedback_eval_state == 1:
            self.publisher.publish(Int16(1))
        else:
            self.publisher.publish(Int16(3))


#rate = rospy.Rate(30)  # 30hz
#while not rospy.is_shutdown():
    #rate.sleep()
if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    session_type=args[1]
    vis = Visualizer(session_type)
    vis.gui.mainloop()


