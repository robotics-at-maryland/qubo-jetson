#!/usr/bin/env python
# Required for ROS to execute file as a Python script

import rospy
from std_msgs.msg import String


class SMACHNode:
    pub = None
    rate = None
    state = 'Initialization'
    gatePassFlag = False

    def __init__(self):
        rospy.init_node('smachNode', anonymous=True)
        self.pub = rospy.Publisher('state', String, queue_size=10)
        self.rate = rospy.Rate(10)   # 10hz


    def initialization(self):
        self.state = 'Initialization'
        rospy.loginfo(self.state)
        self.pub.publish(self.state)
        self.rate.sleep()

        # Dummy condition generator
        condition = raw_input('Initialization successful? Enter Y or N')

        if (condition == 'y'):
            self.state = 'Gate Search'
        else:
            self.state = 'Debug'
    

    def debug(self):
        self.state = 'Debug'
        rospy.loginfo(self.state)
        self.pub.publish(self.state)
        self.rate.sleep()

        # Dummy condition generator
        condition = raw_input('Debug complete? Enter y or n')

        if (condition == 'y'):
            self.state = 'Initialization'
        else:
            self.state = 'Debug'
    

    def gateSearch(self):
        self.state = 'Gate Search'
        rospy.loginfo(self.state)
        self.pub.publish(self.state)
        self.rate.sleep()

        # Dummy condition generator
        condition = raw_input('Gate found? Enter y or n')

        if (condition == 'y'):
            self.state = 'Gate Navigation'
        else:
            self.state = 'Gate Search'
    

    def gateNavigation(self):
        self.state = 'Gate Navigation'
        rospy.loginfo(self.state)
        self.pub.publish(self.state)
        self.rate.sleep()

        # Dummy condition generator
        condition = raw_input('Is the gate passed or lost? Enter p or l')

        if (condition == 'p' and self.gatePassFlag == False):
            self.gatePassFlag = True
            self.state = 'Pole Search'
        elif(condition == 'p' and self.gatePassFlag == True):
            self.state = 'Breach and Shutdown'
        else:
            self.state = 'Gate Search'
    

    def poleSearch(self):
        self.state = 'Pole Search'
        rospy.loginfo(self.state)
        self.pub.publish(self.state)
        self.rate.sleep()

        # Dummy condition generator
        condition = raw_input('Pole found? Enter y or n')

        if (condition == 'y'):
            self.state = 'Pole Navigation'
        else:
            self.state = 'Pole Search'
    

    def poleNavigation(self):
        self.state = 'Pole Navigation'
        rospy.loginfo(self.state)
        self.pub.publish(self.state)
        self.rate.sleep()

        # Dummy condition generator
        condition = raw_input('Is the pole passed or lost? Enter p or l')

        if (condition == 'p'):
            self.state = 'Gate Search'
        else:
            self.state = 'Pole Search'


    def breachAndShutdown(self):
        self.state = 'Breach and Shutdown'
        rospy.loginfo(self.state)
        self.pub.publish(self.state)
        self.rate.sleep()


    def run(self):
        while not rospy.is_shutdown():
            if (self.state == 'Initialization'):
                self.initialization()

            elif (self.state == 'Debug'):
                self.debug()

            elif (self.state == 'Gate Search'):
                self.gateSearch()

            elif (self.state == 'Gate Navigation'):
                self.gateNavigation()

            elif (self.state == 'Pole Search'):
                self.poleSearch()

            elif (self.state == 'Pole Navigation'):
                self.poleNavigation()

            elif (self.state == 'Breach and Shutdown'):
                self.breachAndShutdown()
                break

            else:
                rospy.loginfo('Invalid State!')
                self.pub.publish('Invalid State!')
                self.rate.sleep()
                break

        

if __name__ == '__main__':
    smachNode = SMACHNode()

    try:
        smachNode.run()
    except rospy.ROSInterruptException:
        pass