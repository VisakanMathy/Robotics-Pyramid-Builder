import numpy as np
import math
#scale value:
sc = 1; #this scales all the brick dimensions and the gaps between the bricks

#brick dimensions
xbrick = 0.192; #this is the length of the brick
ybrick = 0.086; #height of brick
zbrick = 0.062; #width of brick

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
print(lay)

RArm = []
LArm = []

#sl = lay[m[0]-1] #m[0] tells us the number of bricks in the base layer. Therefore lay[m[0]] picks coord that number of bricks along the array (we need the m[0] - 1 This will be used to reference to the lh arm pick up position from. 
#sl[0] = sl[0] - (xb + sc*0.1) #this is the pick up position for the lh arm. I have moved the x position back by a brick length + gap. 

#print(sl) #essentially there is a probelm in the sl lines - the sl value is replacing the layer value so that when you do the k loop, the two lh arm values are identical. Idk why this is happening. 

for k in range(0, len(lay)):
    if lay[k][1] < ys: #lay is a list of coords in a list. This is selecting the third (i.e. z) of the k'th coordinate in the list lay. If this value is smaller than half the bottom layer length
        #RArm.append(s) 
        RArm.append(lay[k])
    else:
        #LArm.append(s)
        LArm.append(lay[k])
        
print(RArm)
print(LArm)
