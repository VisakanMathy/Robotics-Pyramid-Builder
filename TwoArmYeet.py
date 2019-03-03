import numpy as np
import math
#scale value:
sc = 1; #this scales all the brick dimensions and the gaps between the bricks

#brick dimensions
xbrick = 0.192; #this is the length of the brick
ybrick = 0.086; #width of brick
zbrick = 0.062; #height of brick

#Scaled brick dimensions
xb = xbrick*sc
yb = ybrick*sc
zb = zbrick*sc

#starting position
xs = 0.512; #this is the starting position along length
ys = 0; #starting position along height
zs = 0.109; #starting position along width

s = [xs, ys, zs] #this is the permanent starting position which is reeferenced as the centre of the brick

#Input for number of layers
x = int(input("how many layers?"))
x1 = list(range(1,x+1))
x1.reverse()
layers = np.asarray(x1)
m = layers.tolist() #this is a list of number of bricks in each layer. e.g. [3, 2, 1] is 3 bricks in base layer, 2 in second etc

lay = [] #this is the matrix containint all the coordinates for the pyramid in x, y, z which corrrespond to length, depth, width with the brick normally orientated
x_structure = round(xs + xb + sc*0.05,4)

for j in range(x):
    z_structure = zs + (x-j)*(zb) + 0.03
    
    if (j+1)%2 == 0: #even layers
        init_list = list(range(int((j+1)/2)))
        rev_list = list(range(int((j+1)/2)))

        for i in range(len(init_list)):
            init_list[i] = i+0.5
            rev_list[i] = -(i+0.5) 
        
        rev_list.reverse()   
        total_list = rev_list+init_list


        
        #bladie
    elif (j+1)%2 == 1: #odd layers
        pos_list = list(range(math.ceil((j+1)/2)))
        neg_list = list(range(-(math.floor((j+1)/2)),0))
        total_list = neg_list + pos_list
        
    for item in total_list:
        y_structure = ys + item*(yb + 0.03)
            
        c = [round(x_structure,4), round(y_structure,4), round(z_structure,4)]
        
        cnew = [] #Creates a new matrix
        cnew = c #Makes cnew equal to the c coordinates
        lay.append(cnew)
        
  
lay.reverse()  
print('lay', lay)

RArm = []
LArm = []
e = 0

for k in range(0, len(lay)): #RULE: YOU NEED TO START WITH RARM FIRST
        if lay[k][1] == ys: #if z is equal to the reference z value, you alternate putting this value in either the right or left ar
            if e == 0: # so the k layer which contains the y = ys brick is always an even or odd layer number. Therefore to append this value into alternating RArm or LArm lists, we change the value of e
                RArm.append(s) 
                RArm.append(lay[k])
                e = 1
            elif e == 1:
                LArm.append(s)
                LArm.append(lay[k])
                e = 0
        elif lay[k][1] < ys and lay[k][1] != 0: #lay is a list of coords in a list. This is selecting the third (i.e. z) of the k'th coordinate in the list lay. If this value is smaller than half the bottom layer length
            RArm.append(s) 
            RArm.append(lay[k])
        elif lay[k][1] > ys and lay[k][1] != 0:
            LArm.append(s)
            LArm.append(lay[k])

print('RArm', RArm)
print('LArm', LArm)