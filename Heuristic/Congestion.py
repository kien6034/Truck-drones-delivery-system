import pandas as pd 

#for zone 1
CONGESTION = [[10, 0.8,0.6,0.4,0.2],  #MorningIn
    [11, 0.35,0.30,0.25,0.20], #'MorningOut'

    [20, 0.5,0.4,0.3,0.2], #'MiddayIn'
    [21, 0.5,0.4,0.3,0.2], #'MiddayOut'
    
    [30, 0.35,0.30,0.25,0.20], #'EveningIn'
    [31, 0.80,0.60,0.40,0.20], #'EveningOut'
 
    [40,0,0,0,0], #'NightIn'
    [41, 0,0,0,0]] #'NightOut'

df = pd.DataFrame(CONGESTION, columns =['TD', 'MT', 'PS', 'TS', 'Ot']) 

TIME_PERIOD = {
    0: (7, 10),
    1: (10, 16),
    2: (16, 19),
    3: (19, 31) #31 equal 7 moring of the next day
}

def calculate_time_period(time):
    
    #morning peak
    if 7 <= time and time < 10:
        return 0
    #midday
    elif 10 <= time and time < 16:
        return 1
    #evening peak
    elif 16 <= time and time < 19:
        return 2
    else:
        return 3
 

def calculate_z1_congestion(f, arc):
    row = int(f) * 2 + int(arc.direction)
    
    col = None

    rType = arc.type
     #primary and secondary
    if rType == "motorway" or rType == "motorway_link" or rType == "trunk" or rType == "trunk_link":
        col = "MT"

     #primary and secondary
    if rType == "primary" or rType == "primary_link" or rType == "secondary" or rType == "secondary_link":
        col = "PS"

    #tertiary and service
    elif rType == "tertiary" or rType == "tertiary_link" or rType == "service" or rType == "service_link":
        col = "TS"
    #other
    else:
        col = "Ot"

    
    #get congestiadon z1 zone
    z1congestion = df.loc[row][col]
  
    return z1congestion