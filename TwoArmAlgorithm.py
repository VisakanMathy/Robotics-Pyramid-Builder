import numpy as np

#brick dimensions
xb = 0.192; #this is the length of the brick
yb = 0.062; #height of brick
zb = 0.086; #width of brick

#starting position
xs = 0; #this is the starting position along length
ys = 0; #starting position along height
zs = 0; #starting position along width

s = [xs, ys, zs] #this is the permanent starting position which is reeferenced as the centre of the brick

#Input for number of layers
x = int(input("how many layers?"))
x1 = list(range(1,x+1))
x1.reverse()
layers = np.asarray(x1)
m = layers.tolist() #this is a list of number of bricks in each layer. e.g. [3, 2, 1] is 3 bricks in base layer, 2 in second etc

lay = [] #this is the matrix containint all the coordinates for the pyramid in x, y, z which corrrespond to length, depth, width with the brick normally orientated

for j in range(0, x): #This loops the inner code from 0-x i.e. 0 to x number of layers
    for i in range(0, m[j]): #this is doing the loop m[J] times (e.g. first time, it will look at the value of m[0] which is the number of bricks in the base layer)
        c = [xs + xb + 0.1, ys + j*(yb) + 0.01, zs + (j/2)*(zb + 0.03)] #This generates the coordinate of the brick depending on the layer value j. this is the changing coord of the centre of the brick on the table. We are also dropping the brick 10cm higher than the layer height needed. 
        cnew = [] #Creates a new matrix
        cnew = c #Makes cnew equal to the c coordinates
        if i in range(1, m[j]): #so i needs to be between 1-m[j] as when i = 0, as this is the first loop, so creates the first recursive value to put values into the next loop
            cnew[2] = cnew[2] + i*(zb + 0.03) #increased z coordinate by width + 3cm gap. This updates the cnew[2] value with the old cnew[2] value (which references the layer shift with j) and some additional shift based on the brick number in that layer i.
            #lay.append(s) #appending starting position first
            lay.append(cnew) #appending new coordinate position         
        else:
            #lay.append(s) #appending first starting position  
            lay.append(cnew) #appending first coordinate position 

print(lay)

RArm = []
LArm = []
sl = lay[m[0]-1] #m[0] tells us the number of bricks in the base layer. Therefore lay[m[0]] picks coord that number of bricks along the array (we need the m[0] - 1 This will be used to reference to the lh arm pick up position from. 

sl[0] = sl[0] - (xb + 0.1) #this is the pick up position for the lh arm. I have moved the x position back by a brick length + gap. 

for k in range(0, len(lay)):
    if lay[k][2] < (m[0]*(zb) + (m[0]-1)*0.03)/2: #lay is a list of coords in a list. This is selecting the third (i.e. z) of the k'th coordinate in the list lay. If this value is smaller than half the bottom layer length
        RArm.append(s) 
        RArm.append(lay[k])
    else:
        LArm.append(sl)
        LArm.append(lay[k])
        
print(RArm)
print(LArm)
