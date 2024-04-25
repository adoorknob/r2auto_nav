import math
import numpy as np

target_error = 0.1
factor = 0.3
lookahead_distance = 0.3

TURTLEBOT_RADIUS = 0.25


# tell robot to keep moving to next cell
# reach next cell then move on to next cell
# v and w can js keep changing ig
    
#generate v and w based on next cell 
# target heading relative to horizontal 
# target heading = atan(nextcell[1]- currcell[1], nextcell[0] - currcell[0])
# target heading - current heading = desired steering angle in radian 
# then do the turn shorter angle stuff 
# turn till abs(desired_steering angle) < math.pi/6 then dont turn 
def target_reached(curr_x, curr_y, target, target_error):
    return (abs(curr_x - target[0]) < target_error and abs(curr_y - target[1]) < target_error)

def avg(list):
    return sum(list)/len(list)

def scale_angle(laser_angle, laserrange):
    return laser_angle * 360 / laserrange

def theory_to_laser(angle, laserrange):
    return round(angle * laserrange / 360 )

# def find_opening(laserrange): V1 HAS PROBLEMS 
#     RightFound = False
#     LeftFound = False
#     np.savetxt('laserrange.txt', laserrange)
#     for i in range(5, int(len(laserrange)/2) - 5):

#         ##### SPKIKE EXTRACTION ######
#         if not RightFound:
#             #check right side
#             right_inner_dist = avg(laserrange[i - 5: i])
#             right_outer_dist = avg(laserrange[i:i+5])
#             # print((right_outer_dist - right_inner_dist) )

#             if (right_outer_dist - right_inner_dist) > 0.2: # if wall is infront of robot
#                 # opening right outer
#                 right_angle = i
#                 RightFound = True
#                 Outer = True
#                 print("Right outer opening Found")
#             elif (right_inner_dist - right_outer_dist) > 0.2:  #if the wall is like beside robot
#                 right_angle = i
#                 RightFound = True
#                 Outer = False
#                 print("Right inner opening Found ")
        
#         if not LeftFound: 
#             #check left side 
#             left_inner_dist = avg(laserrange[-i-5:-i])
#             left_outer_dist = avg(laserrange[-i-10: -i-5]) 
#             # print('Left wall dist: ', left_outer_dist- left_inner_dist)
#             if not LeftFound and (left_outer_dist- left_inner_dist) > 0.2:
#                 left_angle = i 
#                 LeftFound = True
#                 Outer = True
#                 print("LeftFound")
#             elif (left_inner_dist - left_outer_dist) > 0.2:  #if the wall is like beside robot
#                 left_angle = i
#                 LeftFound = True
#                 Outer = False
#                 print("Right inner opening Found ")
        


#     if LeftFound and RightFound:
#         if left_angle > right_angle: # GO RIGHT
#             laser_angle = right_angle
#             if Outer:
#                 wall_dist = right_outer_dist
#             else:
#                 wall_dist = right_inner_dist
#         else: #GO RIGHT
#             laser_angle = len(laserrange) - left_angle
#             if Outer:
#                 wall_dist = left_outer_dist
#             else:
#                 wall_dist = left_inner_dist
    
#     elif LeftFound:
#         laser_angle = len(laserrange) - left_angle
#         if Outer:
#             wall_dist = left_outer_dist
#         else:
#             wall_dist = left_inner_dist
    
#     elif RightFound:
#         laser_angle = right_angle
#         if Outer:
#             wall_dist = right_outer_dist
#         else:
#             wall_dist = right_inner_dist

#     else:
#         print("knn find opening also cannot")
#         print("opening straight ahead?")
#         # laser_angle = 0
#         # wall_dist = laserrange[0]
#         return 0
    
#     actl_angle = scale_angle(laser_angle, len(laserrange))
#     add_angle = math.atan(TURTLEBOT_RADIUS/wall_dist)
    

#     if actl_angle > 180:
#         rotangle = actl_angle + add_angle
#     else:
#         rotangle = actl_angle - add_angle
    
#     # if rotangle 

#     return rotangle


def find_opening(laserrange): #FIXED V1 SHOULD COOK
    RightFound = False
    LeftFound = False
    np.savetxt('laserrange.txt', laserrange)
    for i in range(5, int(len(laserrange)/2) - 5):

        ##### SPKIKE EXTRACTION ######
        if not RightFound:
            #check right side
            right_inner_dist = avg(laserrange[i - 5: i])
            right_outer_dist = avg(laserrange[i:i+5])
            # print((right_outer_dist - right_inner_dist) )

            if (right_outer_dist - right_inner_dist) > 0.2: # if wall is infront of robot
                # opening right outer
                right_angle = i
                RightFound = True
                Outer = True
                print("Right outer opening Found")
            elif (right_inner_dist - right_outer_dist) > 0.2:  #if the wall is like beside robot
                right_angle = i
                RightFound = True
                Outer = False
                print("Right inner opening Found ")
        
        if not LeftFound: 
            #check left side 
            left_inner_dist = avg(laserrange[-i-5:-i])
            left_outer_dist = avg(laserrange[-i-10: -i-5]) 
            # print('Left wall dist: ', left_outer_dist- left_inner_dist)
            if (left_outer_dist- left_inner_dist) > 0.2:
                left_angle = i 
                LeftFound = True
                Outer = True
                print("Left outer opening Found ")
            elif (left_inner_dist - left_outer_dist) > 0.2:  #if the wall is like beside robot
                left_angle = i
                LeftFound = True
                Outer = False
                print("Left inner opening Found ")
        


    if LeftFound and RightFound:
        if left_angle > right_angle: 
            # GO RIGHT
            print("Choose Right")
            laser_angle = right_angle
            if Outer:
                wall_dist = right_inner_dist
            else:
                wall_dist = right_outer_dist
        else: #GO RIGHT
            print("Choose Left")
            laser_angle = len(laserrange) - left_angle
            if Outer:
                wall_dist = left_inner_dist
            else:
                wall_dist = left_outer_dist
    
    elif LeftFound:
        laser_angle = len(laserrange) - left_angle
        if Outer:
            wall_dist = left_inner_dist
        else:
            wall_dist = left_outer_dist
    
    elif RightFound:
        laser_angle = right_angle
        if Outer:
            wall_dist = right_inner_dist
        else:
            wall_dist = right_outer_dist

    else:
        print("knn find opening also cannot")
        print("opening straight ahead?")
        # laser_angle = 0
        # wall_dist = laserrange[0]
        return 0
    
    actl_angle = scale_angle(laser_angle, len(laserrange))
    add_angle = math.degrees(math.atan(TURTLEBOT_RADIUS/wall_dist))
    print("Actl angle", actl_angle)
    print("Wall dist", wall_dist)
    print("Add Angle", add_angle)
    
    if (RightFound and Outer) or (LeftFound and not Outer):
        rotangle = actl_angle + add_angle
    else:
        rotangle = actl_angle - add_angle
    
    # if rotangle 
    return rotangle

def direct_opening(laserrange, preferred_angle): #V2
    RightFound = False
    LeftFound = False
    max_laser = len(laserrange)
    np.savetxt('laserrange.txt', laserrange)
    for i in range(5, int(len(laserrange)/2) - 5):

        ##### SPKIKE EXTRACTION ######
        if not RightFound:
            #check right side
            right_inner_dist = avg(laserrange[i - 5: i])
            right_outer_dist = avg(laserrange[i:i+5])
            # print((right_outer_dist - right_inner_dist) )

            if (right_outer_dist - right_inner_dist) > 0.2: # if wall is infront of robot
                # opening right outer
                right_angle = i
                RightFound = True
                Outer = True
                print("Right outer opening Found")
            elif (right_inner_dist - right_outer_dist) > 0.2:  #if the wall is like beside robot
                right_angle = i
                RightFound = True
                Outer = False
                print("Right inner opening Found ")
        
        if not LeftFound: 
            #check left side 
            left_inner_dist = avg(laserrange[-i-5:-i])
            left_outer_dist = avg(laserrange[-i-10: -i-5]) 
            # print('Left wall dist: ', left_outer_dist- left_inner_dist)
            if (left_outer_dist- left_inner_dist) > 0.2:
                left_angle = i 
                LeftFound = True
                Outer = True
                print("Left outer opening Found ")
            elif (left_inner_dist - left_outer_dist) > 0.2:  #if the wall is like beside robot
                left_angle = i
                LeftFound = True
                Outer = False
                print("Left inner opening Found ")
        


    if LeftFound and RightFound:
        print("Preferred Angle", preferred_angle)
        print("Left diff:", abs(scale_angle(-left_angle, max_laser) - preferred_angle))
        print("Right Diff:", abs(scale_angle(right_angle, max_laser) - preferred_angle))
        if abs(scale_angle(-left_angle, max_laser) - preferred_angle) > abs(scale_angle(right_angle, max_laser) - preferred_angle): 
            # GO RIGHT
            print("Choose Right")
            laser_angle = right_angle
            if Outer:
                wall_dist = right_inner_dist
            else:
                wall_dist = right_outer_dist
        else: #GO RIGHT
            print("Choose Left")
            laser_angle = len(laserrange) - left_angle
            if Outer:
                wall_dist = left_inner_dist
            else:
                wall_dist = left_outer_dist
    
    elif LeftFound:
        laser_angle = len(laserrange) - left_angle
        if Outer:
            wall_dist = left_inner_dist
        else:
            wall_dist = left_outer_dist
    
    elif RightFound:
        laser_angle = right_angle
        if Outer:
            wall_dist = right_inner_dist
        else:
            wall_dist = right_outer_dist

    else:
        print("knn find opening also cannot")
        print("opening straight ahead?")
        # laser_angle = 0
        # wall_dist = laserrange[0]
        return 0
    
    actl_angle = scale_angle(laser_angle, max_laser)
    add_angle = math.degrees(math.atan(TURTLEBOT_RADIUS/wall_dist))
    print("Actl angle", actl_angle)
    print("Wall dist", wall_dist)
    print("Add Angle", add_angle)
    
    if (RightFound and Outer) or (LeftFound and not Outer):
        rotangle = actl_angle + add_angle
    else:
        rotangle = actl_angle - add_angle
    
    # if rotangle 
    return rotangle


def find_opening_V3(laserrange, preferred_angle): #V3
    #doesnt just find closest opening checks path then finds opening for that path
    #priority is on the path angle instead of the nearest opening 
    RightFound = False
    LeftFound = False
    np.savetxt('laserrange.txt', laserrange)
    if preferred_angle < 0:
        preferred_angle += 360
    print("Preferred angle", preferred_angle)
    index = theory_to_laser(preferred_angle, len(laserrange))
    print("Index:", index)
    #translate list so that preferred _angle is at 0 degrees for checking 
    cut = laserrange[:index]
    remaining = laserrange[index:]
    laserrange = np.concatenate((cut, remaining))
    
    for i in range(5, int(len(laserrange)/2) - 5):

        ##### SPKIKE EXTRACTION ######
        if not RightFound:
            #check right side
            right_inner_dist = avg(laserrange[i - 5: i])
            right_outer_dist = avg(laserrange[i:i+5])
            # print((right_outer_dist - right_inner_dist) )

            if (right_outer_dist - right_inner_dist) > 0.2: # if wall is infront of robot
                # opening right outer
                right_angle = i
                RightFound = True
                Outer = True
                print("Right outer opening Found")
            elif (right_inner_dist - right_outer_dist) > 0.2:  #if the wall is like beside robot
                right_angle = i
                RightFound = True
                Outer = False
                print("Right inner opening Found ")
        
        if not LeftFound: 
            #check left side 
            left_inner_dist = avg(laserrange[-i-5:-i])
            left_outer_dist = avg(laserrange[-i-10: -i-5]) 
            # print('Left wall dist: ', left_outer_dist- left_inner_dist)
            if (left_outer_dist- left_inner_dist) > 0.2:
                left_angle = i 
                LeftFound = True
                Outer = True
                print("Left outer opening Found ")
            elif (left_inner_dist - left_outer_dist) > 0.2:  #if the wall is like beside robot
                left_angle = i
                LeftFound = True
                Outer = False
                print("Left inner opening Found ")
        


    if LeftFound and RightFound:
        if left_angle > right_angle: 
            # GO RIGHT
            print("Choose Right")
            laser_angle = right_angle
            if Outer:
                wall_dist = right_inner_dist
            else:
                wall_dist = right_outer_dist
        else: #GO RIGHT
            print("Choose Left")
            laser_angle = len(laserrange) - left_angle
            if Outer:
                wall_dist = left_inner_dist
            else:
                wall_dist = left_outer_dist
    
    elif LeftFound:
        laser_angle = len(laserrange) - left_angle
        if Outer:
            wall_dist = left_inner_dist
        else:
            wall_dist = left_outer_dist
    
    elif RightFound:
        laser_angle = right_angle
        if Outer:
            wall_dist = right_inner_dist
        else:
            wall_dist = right_outer_dist

    else:
        print("knn find opening also cannot")
        print("opening straight ahead?")
        # laser_angle = 0
        # wall_dist = laserrange[0]
        return 0
    print("laser_angle", laser_angle)
    if index > 180:
        index -= 360
    laser_angle += index #translate it back 
    actl_angle = scale_angle(laser_angle, len(laserrange))
    if actl_angle > 180:
        actl_angle -= 360
    add_angle = math.degrees(math.atan(TURTLEBOT_RADIUS/wall_dist))
    print("Actl angle", actl_angle)
    print("Wall dist", wall_dist)
    print("Add Angle", add_angle)
    
    if (RightFound and Outer) or (LeftFound and not Outer):
        rotangle = actl_angle + add_angle
    else:
        rotangle = actl_angle - add_angle
    
    # if rotangle 
    return rotangle





def find_next_point(current_x, current_y, path): 
    for i in range(1, len(path)):
        x = path[-i][0]
        y = path[-i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance > distance:
            closest_point = (x, y)
            index = i
            break
    return closest_point, path[-index:]    


def pick_direction(target_x, target_y, curr_x, curr_y, curr_heading):
    target_heading =  math.atan2(target_y - curr_y, target_x - curr_x)
    desired_steering_angle = target_heading - curr_heading
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    return desired_steering_angle
    
    v = v_speed
    w = desired_steering_angle * factor
    if abs(desired_steering_angle) > math.pi/2:
        v = 0.0
    elif desired_steering_angle > math.pi/4:
        w = w_speed
    elif desired_steering_angle < -math.pi/4:
        w = w_speed * -1
    elif desired_steering_angle < 0:
        w *= -1 
    return v, w
