import rospy
from std_msgs.msg import Float64MultiArray
import rosgraph
import rostopic
import re

PI = 3.141592
D2R = PI / 180.0
R2D = 180.0 / PI
Joint_errors = []
joint_positions = []
def topic_scan():
    
    master = rosgraph.Master('/rostopic')
    pubs, subs = rostopic.get_topic_list(master=master)

    for topics in subs:
        name = topics[0]
        if "elfin" in name and "command" in name:
            #print("Command Found: ", name)
            return name
    return False

def Joint_Traj_planner(current, Target, Resolution_scaler=1):  #Current_positions, Targets, fine grain steps
    traj = []
    difference = [a_i - b_i for a_i, b_i in zip(Target, current)] #calculate difference between target and current values
    abs_diff = [abs(ele) for ele in difference]
    max_diff = round( round( max(abs_diff) ) * Resolution_scaler ) #Take the maximum value and multiply with scaler 
                                                 #to set equal number of waypoints for each joint
    print(difference)
    print(abs_diff)
    print(max_diff)
    j0 = current[0]
    j1 = current[1]
    j2 = current[2]
    j3 = current[3]
    j4 = current[4]
    j5 = current[5]

    for i in range(0,max_diff):
        j0 = j0 + ( (Target[0]-current[0])/max_diff )
        j1 = j1 + ( (Target[1]-current[1])/max_diff )
        j2 = j2 + ( (Target[2]-current[2])/max_diff )
        j3 = j3 + ( (Target[3]-current[3])/max_diff )
        j4 = j4 + ( (Target[4]-current[4])/max_diff )
        j5 = j5 + ( (Target[5]-current[5])/max_diff )
        traj.append([j0,j1,j2,j3,j4,j5])
    
    return traj, len(traj) #return trajectory and the number of waypoints generated
    

def Joint_Traj_Executor(Joint_values, waypoints, count,pub, pub_topic, sleep_time):
    #global Joint_errors
    D2R = 3.141592 / 180.0

    print("Publishing values to:", pub_topic)
    print()

    for i in range (0, count):

        Joint_values.data[0] = waypoints[i][0]*D2R
        Joint_values.data[1] = waypoints[i][1]*D2R
        Joint_values.data[2] = waypoints[i][2]*D2R
        Joint_values.data[3] = waypoints[i][3]*D2R
        Joint_values.data[4] = waypoints[i][4]*D2R
        Joint_values.data[5] = waypoints[i][5]*D2R
        print("Publishing waypoint",i+1,"of",count,":", Joint_values.data)
        pub.publish(Joint_values)
        rospy.sleep(sleep_time)
#        while (max(Joint_errors) > 1.5): wait while angle error is greater than 1.5 degrees
#            print(max(Joint_errors), "is greater than 1.5")


def Elfin_updates(Info): #Callback function which is called when a new message of type Float64MultiArray is received by the subscriber.
    global Joint_errors, joint_positions
    joint_positions = [ Info.data[19]*R2D, Info.data[20]*R2D, Info.data[21]*R2D, Info.data[22]*R2D, Info.data[23]*R2D, Info.data[24]*R2D ]

    #Joint_errors = [ Info.data[31]*R2D, Info.data[32]*R2D, Info.data[33]*R2D, Info.data[34]*R2D, Info.data[35]*R2D, Info.data[36]*R2D ]
    #Joint_errors =  [abs(ele) for ele in Joint_errors]
    
    #print('Joint_errors: ',Joint_errors)

def talker(pub_topic):
    global joint_positions
    sub_topic = pub_topic.replace("command", "SaveData" )
    pub = rospy.Publisher(pub_topic, Float64MultiArray, queue_size=10)
    sub = rospy.Subscriber(sub_topic, Float64MultiArray, Elfin_updates)
    rospy.init_node('Commander', anonymous=False)
    arr = Float64MultiArray()
    #rate = rospy.Rate(10) # 10hz
    c=0

    Mode = input("Select the mode Enter 1 for Task space commands, Enter 2 for Joint commands, 3 for Joint waypoints: ").rstrip().replace(",", " " ).replace(";", " " )     # Get the input from the user.
    if(Mode.isnumeric() == False):
        print("Invalid Mode!")
        return
    else:
        Mode = int(Mode)
    if(Mode == 1):

        while True:
            val = input("Set your XYZ and [RPY] values with space between each.: ").rstrip().replace(",", " " ).replace(";", " " )     # Get the input from the user.

            if(len(val) != 0 and bool(re.match('^[0-9\-\.\ ]*$',val)) == True):
                arr.data = list(map(float, val.split(' ')))
            else:
                print("Invalid Data")
                break

            if(len(arr.data) == 6):
                arr.data[3]=arr.data[3]*D2R
                arr.data[4]=arr.data[4]*D2R
                arr.data[5]=arr.data[5]*D2R
                arr.data.append(Mode)
                print("Publishing: ", arr.data, " to", pub_topic)
                pub.publish(arr)

            elif(len(arr.data) == 3):
                arr.data.append(0.0)
                arr.data.append(0.0)
                arr.data.append(0.0)
                arr.data.append(Mode)
                print("Publishing: ", arr.data, " to", pub_topic)
                pub.publish(arr)
            else:
                print("Invalid XYZ [RPY] Data Length! Not publishing")
            print()

            c+=1

    elif(Mode == 2):
        while True:
            val = input("Enter Joint values in Degrees for all six joints.: ").rstrip().replace(",", " " ).replace(";", " " )     # Get the input from the user.

            if(len(val) != 0 and bool(re.match('^[0-9\-\.\ ]*$',val)) == True):
                arr.data = list(map(float, val.split(' ')))
            else:
                print("Invalid Data")
                break

            if(len(arr.data) == 6):
                arr.data[0]=arr.data[0]*D2R
                arr.data[1]=arr.data[1]*D2R
                arr.data[2]=arr.data[2]*D2R
                arr.data[3]=arr.data[3]*D2R
                arr.data[4]=arr.data[4]*D2R
                arr.data[5]=arr.data[5]*D2R
                arr.data.append(Mode)
                print("Publishing: ", arr.data, " to", pub_topic)
                pub.publish(arr)

            else:
                print("Invalid (Joint angles) Data Length! Not publishing")
                print("Retry!!!")
            print()

            c+=1
        #rospy.spin()     # If we press control + C, the node will stop.

    elif(Mode == 3):
        while True:
            val = input("Enter Joint values in Degrees for all six joints.: ").rstrip().replace(",", " " ).replace(";", " " )     # Get the input from the user.

            if(len(val) != 0 and bool(re.match('^[0-9\-\.\ ]*$',val)) == True):
                arr.data = list(map(float, val.split(' ')))
            else:
                print("Invalid Data")
                break


            if(len(arr.data) == 6):
                waypoints, count = Joint_Traj_planner(joint_positions, arr.data, 1) #current, target, resolution_multiplier
                arr.data.append(Mode)
                Joint_Traj_Executor(arr, waypoints, count, pub, pub_topic, 0.03)

            else:
                print("Invalid (Joint angles) Data Length! Not publishing")
                print("Retry!!!")
            print()

            c+=1
        #rospy.spin()     # If we press control + C, the node will stop.
cmd_topic = topic_scan()

talker(cmd_topic)
