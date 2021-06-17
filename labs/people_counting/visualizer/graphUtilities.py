#functions for creating ellipsoids to represent tracks
import math, time
import numpy as np

#helper funtions
def getSphereVertexes(xRadius=0.75, yRadius=0.75, zRadius=1.5,xc=0,yc=0,zc=0,stacks=6,sectors=6):
    stackStep = math.pi/stacks
    sectorStep = 2*math.pi/sectors
    verts = np.empty((3, (stacks+1)*sectors))
    for i in range(0,stacks+1):
        stackAngle = math.pi/2 - i*stackStep
        xr = xRadius*math.cos(stackAngle)
        yr = yRadius*math.cos(stackAngle)
        z = zRadius*math.sin(stackAngle) + zc
        for j in range(0,sectors):
            sectorAngle = j*sectorStep
            #vertex position
            x = xr*math.cos(sectorAngle) + xc
            y = yr*math.sin(sectorAngle) + yc
            verts[0,i*stacks+j] = x
            verts[1,i*stacks+j] = y
            verts[2,i*stacks+j] = z
    return verts

def getSphereTriangles(verts, stacks=6, sectors=6):
    trigVerts = np.empty((((stacks)*sectors*2),3,3))
    ind = 0
    for i in range(0,stacks):
        k1 = i*sectors
        k2 = k1 + sectors
        for j in range(0,sectors):
            k1v = k1+j
            k1v2 = k1+((j+1)%sectors)
            k2v = k2+j
            k2v2 = k2+((j+1)%sectors)
            if (i!=0):
                trigVerts[ind,0,:] = verts[:,k1v]
                trigVerts[ind,1,:] = verts[:,k2v]
                trigVerts[ind,2,:] = verts[:,k1v2]
                ind += 1
            if(i!=stacks-1):
                trigVerts[ind,0,:] = verts[:,k1v2]
                trigVerts[ind,1,:] = verts[:,k2v]
                trigVerts[ind,2,:] = verts[:,k2v2]
                ind += 1
    return trigVerts

def getSphereMesh(xRadius=0.75, yRadius=0.75, zRadius=0.75,xc=0,yc=0,zc=0,stacks=6,sectors=6,bench=0):
    if (bench):
        startTime = int(round(time.time()*1000))
    verts = getSphereVertexes(xRadius=xRadius,yRadius=yRadius,zRadius=zRadius,xc=xc,yc=yc,zc=zc,stacks=stacks,sectors=sectors)
    if(bench):
        endTime = int(round(time.time()*1000))
        timeSpent = endTime - startTime
        print('Sphere Mesh Bench: ', timeSpent,' ms')
    return getSphereTriangles(verts=verts, stacks=stacks, sectors=sectors)

def getBoxVertices(xl,yl,zl,xr,yr,zr):
    verts = np.zeros((8,3))
    verts[0,:] = [xl,yl,zl]
    verts[1,:] = [xr,yl,zl]
    verts[2,:] = [xl,yr,zl]
    verts[3,:] = [xr,yr,zl]
    verts[4,:] = [xl,yl,zr]
    verts[5,:] = [xr,yl,zr]
    verts[6,:] = [xl,yr,zr]
    verts[7,:] = [xr,yr,zr]
    return verts

def getBoxLinesFromVerts(verts):
    lines = np.zeros((24,3))
    #outer loop gets the key vertexes
    #keyVerts = [0,3,5,6]
    #v0
    lines[0]= verts[0]
    lines[1]= verts[1]
    lines[2]= verts[0]
    lines[3]= verts[2]
    lines[4]= verts[0]
    lines[5]= verts[4]
    #v3
    lines[6]= verts[3]
    lines[7]= verts[1]
    lines[8]= verts[3]
    lines[9]= verts[2]
    lines[10]=verts[3]
    lines[11]=verts[7]
    #v5
    lines[12]=verts[5]
    lines[13]=verts[4]
    lines[14]=verts[5]
    lines[15]=verts[7]
    lines[16]=verts[5]
    lines[17]=verts[1]
    #v6
    lines[18]=verts[6]
    lines[19]=verts[2]
    lines[20]=verts[6]
    lines[21]=verts[4]
    lines[22]=verts[6]
    lines[23]=verts[7]
    return lines

def getBoxLines(xl,yl,zl,xr,yr,zr):
    verts = getBoxVertices(xl,yl,zl,xr,yr,zr)
    return getBoxLinesFromVerts(verts)

def getBoxLinesCoords(x,y,z,xrad=0.25,yrad=0.25,zrad=0.5):
    xl=x-xrad
    xr=x+xrad
    yl=y-yrad
    yr=y+yrad
    zl=z-zrad
    zr=z+zrad
    verts = getBoxVertices(xl,yl,zl,xr,yr,zr)
    return getBoxLinesFromVerts(verts)

def getSquareLines(xl,yL,xr,yr,z):
    verts = np.zeros((5,3))
    verts[0,:] = [xl,yL,z]
    verts[1,:] = [xr,yL,z]
    verts[2,:] = [xr,yr,z]
    verts[3,:] = [xl,yr,z]
    verts[4,:] = [xl,yL,z]
    return verts

#function to rotate a point [x,y,z] about the x axis by an angle theta
def rotX(x,y,z,theta):
    
    theta = np.deg2rad(theta) #convert to radians
    Rx = np.matrix([[ 1, 0           , 0           ],
                   [ 0, math.cos(theta),-math.sin(theta)],
                   [ 0, math.sin(theta), math.cos(theta)]])
  
    #Ry = np.matrix([[ math.cos(theta), 0, math.sin(theta)],
    #               [ 0           , 1, 0           ],
    #               [-math.sin(theta), 0, math.cos(theta)]])
    #
    #Rz = np.matrix([[ math.cos(theta), -math.sin(theta), 0 ],
    #               [ math.sin(theta), math.cos(theta) , 0 ],
    #               [ 0           , 0            , 1 ]])
    #print('passed points = ' + str(x) + ', '+ str(y) + ', '+ str(z))
    target =  np.array([x,y,z])
    #print('target array = ',target)
    rotTarget = Rx*target
    #print('rotated target array = ',rotTarget)
    return rotTarget[0],rotTarget[1],rotTarget[2]

#def getSquare(verts):
#    lines = np.zeros((4,3))
#    lines[0]= verts[0]
#    lines[1]= verts[1]
#    lines[2]= verts[0]
#    lines[3]= verts[2]