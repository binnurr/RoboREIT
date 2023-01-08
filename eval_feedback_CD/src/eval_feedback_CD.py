#!/usr/bin/env python

from selenium import webdriver
from selenium.common.exceptions import TimeoutException
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from selenium.webdriver.common.by import By
import time
import random
import rospy
from std_msgs.msg import String
import json
import os
from rospkg import RosPack


class WebDriver:
    def __init__(self, verbose=False):
        self.verbose = verbose
        options = webdriver.ChromeOptions()
        options.add_argument("headless")
        self.driver = webdriver.Chrome(os.path.join(RosPack().get_path("fsc_py_planner"), "utils", "chromedriver"),
                                       chrome_options=options)
        self.driver.get("http://www.interviewsim.com.s3-website.us-east-2.amazonaws.com/")
        timeout = 1.0
        try:
            WebDriverWait(self.driver, timeout).until(lambda x: x.find_elements_by_class_name("enchantment-link"))
        except TimeoutException:
            print("Timed out waiting for page to load")
        self.selected_questions = {}
        self.feedback_dict = {}

    def run_mock(self, selected_questions=None):
        # Obtain button by link text and click.
        elements_link = self.driver.find_elements_by_class_name("enchantment-link")
        if self.verbose:
            print("ROBOT: Hello! I am Nao. I am your robotic client. I will help you "
                  "to improve your interview skills. In this interview you will be "
                  "talking to Mary from Cool Ski Resorts.")
        turn = 0
        while len(elements_link) == 3:
            if self.verbose:
                print("Select one of the below:")
            for elem in elements_link:
                if self.verbose:
                    print(elem.text)
                    print("************************")
            if selected_questions and (turn in selected_questions):
                self.selected_questions[turn] = selected_questions[turn]
            else:
                self.selected_questions[turn] = random.choice([1, 2, 3])
            elements_link[self.selected_questions[turn] - 1].click()
            #time.sleep(0.1)
            timeout = 1.0
            try:
                WebDriverWait(self.driver, timeout).until(lambda x: x.find_elements_by_tag_name("tw-hook"))
            except TimeoutException:
                print("Timed out waiting for page to load")
            elements_link = self.driver.find_elements_by_class_name("enchantment-link")
            temps = self.driver.find_elements_by_tag_name("tw-hook")
            robot_text = temps[1].text
            if self.verbose:
                print("ROBOT: ", robot_text)
            turn += 1

        print("mock finished")

    def analyze_feedback(self):
        self.driver.find_elements_by_tag_name("tw-link")[0].click()
        timeout = 1.0
        try:
            WebDriverWait(self.driver, timeout).until(lambda x: x.find_elements_by_tag_name("tw-expression"))
        except TimeoutException:
            print("Timed out waiting for page to load")
        tt = self.driver.find_elements_by_tag_name("tw-expression")
        ques_count = int(len(tt) / 6 - 1)
        for i in range(ques_count):
            self.feedback_dict[i] = tt[3 * (ques_count + 1) + 3 + i * 3].text
        if self.verbose:
            print(self.feedback_dict)

    def close(self):
        self.driver.close()


class SearchCorrectQues:
    def __init__(self):
        self.sub = rospy.Subscriber("selected_ques", String, self.selected_ques_callback, queue_size=100)
        self.correct_choices_pub = rospy.Publisher('correct_choices', String, queue_size=1)
        self.selected_questions = {}
        self.turn = 0
        self.correct_choices = {}

    def search_correct_ques(self):
        turn = len(self.selected_questions) - 1
        wd = WebDriver(verbose=False)
        wd.run_mock(self.selected_questions)
        wd.analyze_feedback()
        try:
            if wd.feedback_dict[turn] != '':
                other_aval_ques = [1, 2, 3]
                other_aval_ques.remove(wd.selected_questions[turn])
                for oaq in other_aval_ques:
                    copy_selected_questions = wd.selected_questions.copy()
                    copy_selected_questions[turn] = oaq
                    temp_wd = WebDriver(verbose=False)
                    temp_wd.run_mock(selected_questions=copy_selected_questions)
                    temp_wd.analyze_feedback()
                    if temp_wd.feedback_dict[turn] == '':
                        self.correct_choices[turn] = oaq
                        break
                    temp_wd.close()
            else:
                self.correct_choices[turn] = self.selected_questions[turn]
            if turn not in self.correct_choices:
                self.correct_choices[turn] = -1
        finally:
            wd.close()

    def selected_ques_callback(self, msg):
        self.selected_questions[self.turn] = int(msg.data)
        print("processing for ", self.selected_questions, self.turn)
        try:
            self.search_correct_ques()
        except Exception as e:
            print(e)
            print("assigning correct choice randomly")
            self.correct_choices[self.turn] = random.choice([1, 2, 3])
        encoded_choices_str = json.dumps(self.correct_choices)
        self.correct_choices_pub.publish(String(encoded_choices_str))
        print("analysis completed")
        print(self.correct_choices)
        self.turn += 1


if __name__ == '__main__':
    rospy.init_node('eval_feedback_CD', anonymous=True)
    search_correct_ques = SearchCorrectQues()
    rospy.spin()
        
    
