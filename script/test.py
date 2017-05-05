#!/usr/bin/env python
from sslib import *
from VU_filter import *


if __name__ == '__main__':
    f = open('ground_truth.txt','r')
    l = array([ map(float,line.split(' ')) for line in f if line.strip() != "" ])
    num = 1
    aa = l[:,19:22]
    rr = l[:,22:25]
    l = l[0:-1:num,:]
    
    N = 6000/num
    p, qr, v, a, r, q  = l[0:N,2:5], l[0:N,5:9], l[0:N,9:12], l[0:N,19:22], l[0:N,22:25], l[0:N,15:19]
    a = array([mean(aa[i*num:(i+1)*num,:], axis=0) for i in xrange(0,p.shape[0]) ])
    r = array([mean(rr[i*num:(i+1)*num,:], axis=0) for i in xrange(0,p.shape[0]) ])
    
    #UWB measurement
    #y = array([[cv2.norm(p[i]-uwbanchor[i%4])] for i in xrange(0,N)])
    #n = random.randn(N,1)*0.1
    #uwbmeasure = y + n
    
    #Vision measurement
    #visionmeasure = int32(array([array([ dot(K, p[j]-visionanchor[i]) for i in xrange(4)]).reshape((8)) for j in xrange(N)]) + random.randn(N,8))
    #print visionmeasure.shape
    
    
    xe = zeros((N,11))
    xe[0,2] = 0.27
    xe[0,6] = 1
    Q[ 0:3,  0:3] =  0.98*eye(3)#*10
    Q[ 3:7,  3:7] =  0.01*eye(4)#*10
    Q[ 7:9,  7:9] =  0.81*eye(2)#/2
    Q[  9 ,   9 ] =  900#/2
    Q[ 10 ,  10 ] =  0.000000001
    #1.28 0.81 400 num = 2
    #0.98 0.81,900 num =1
    #0.5 0.01 100 num = 10
    Q = Q*7
    uwb = UWBLocation(1.0/100) 
    uwb.setQ(Q)   
    vision = VisionlLocation(1.0/100, K) 
    vision.setQ(Q)
    
    timer = sstimer()
    timer.start()
    uwbcount = 0

    for i in xrange(0, N-1):
        print i
        if i%2 == 0:
            uwbdis = linalg.norm(p[i]-uwbanchor[uwbcount%4]) + random.rand(1)*0.1     
            xe[i+1], _ = uwb.locate(xe[i], Q, 1.0/100,uwbdis, uwbanchor[uwbcount%4], q[i], a[i], r[i])
            uwbcount = uwbcount + 1
        else:
            visionpoints = array([dot(K, p[i]-visionanchor[j]) for j in xrange(4)]).reshape((8)) + random.rand(8)*10
            xe[i+1], _ = vision.locate(xe[i], Q, 1.0/100, visionpoints, visionanchor, q[i], a[i], r[i])
    
    print "Accuracy:", linalg.norm(xe[:,0:3]-p)
    print "std:", std(xe[:,0:3]-p)
    print "Time: ", timer.end()
    
    
    fig1 = plt.figure()
    ax = fig1.add_subplot(121, projection='3d')
    ax.plot(uwbanchor[:,0],uwbanchor[:,1],uwbanchor[:,2],marker='o',linewidth=3)
    ax.plot(visionanchor[:,0],visionanchor[:,1], visionanchor[:,2], marker='o',linewidth=3)
    ax.plot(xe[:,0], xe[:,1], xe[:,2])
    ax.plot(p[:,0], p[:,1], p[:,2])
    
    ax = fig1.add_subplot(122)
    lx,=ax.plot(abs(xe[:,0]-p[:,0]),color = 'red')
    ly,=ax.plot(abs(xe[:,1]-p[:,1]),color = 'blue')
    lz,=ax.plot(abs(xe[:,2]-p[:,2]),color = 'black')
    plt.legend([lx,ly,lz], ['x', 'y', 'z'])
    plt.title('error of position')
    
    
    fig = plt.figure()
    ax = fig.add_subplot(321)
    lx,=ax.plot(v[:,0],color = 'red')
    ly,=ax.plot(v[:,1],color = 'blue')
    lz,=ax.plot(v[:,2],color = 'black') 
    plt.legend([lx,ly,lz], ['x', 'y', 'z'])
    plt.title('real vel')
    ax = fig.add_subplot(323)
    lx,=ax.plot(xe[:,7],color = 'red')
    ly,=ax.plot(xe[:,8],color = 'blue')
    lz,=ax.plot(xe[:,9],color = 'black') 
    plt.legend([lx,ly,lz], ['x', 'y', 'z'])
    plt.title('est vel')

    ax = fig.add_subplot(3,2,2)
    ax.plot(q[:,0],color = 'red')
    ax.plot(q[:,1],color = 'blue')
    ax.plot(q[:,2],color = 'black')
    ax.plot(q[:,3],color = 'yellow') 
    plt.legend(['x', 'y', 'z', 'w'])
    plt.title('real quaternion')
    ax = fig.add_subplot(3,2,4)
    ax.plot(xe[:,3],color = 'red')
    ax.plot(xe[:,4],color = 'blue')
    ax.plot(xe[:,5],color = 'black')
    ax.plot(xe[:,6],color = 'yellow') 
    plt.legend(['x', 'y', 'z', 'w'])
    plt.title('est quaternion')

    ax = fig.add_subplot(3,2,5)
    ax.plot(r[:,0],color = 'red')
    ax.plot(r[:,1],color = 'blue')
    ax.plot(r[:,2],color = 'black') 
    plt.legend([lx,ly,lz], ['x', 'y', 'z'])
    plt.title('real rate')

    ax = fig.add_subplot(3,2,6)
    ax.plot(a[:,0],color = 'red')
    ax.plot(a[:,1],color = 'blue')
    ax.plot(a[:,2],color = 'black') 
    plt.legend([lx,ly,lz], ['x', 'y', 'z'])
    plt.title('real acc')

    plt.show()
    
else:
    pass
