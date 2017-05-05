from sslib import *
from VU_filter import *

if __name__=='__main__':
    f = open('ground_truth.txt','r')
    l = array([ map(float,line.split(' ')) for line in f if line.strip() != "" ])
    num = 10
    l = l[0:-1:num,:]
    N = 1000/num
    p, qr, v, a, r, q  = l[0:N,2:5], l[0:N,5:9], l[0:N,9:12], l[0:N,19:22], l[0:N,22:25], l[0:N,15:19]
    #q = column_stack((q[:,1:4],q[:,0]))
    #a[:,0],a[:,1]= -a[:,0], -a[:,1]
    
    y = array([[linalg.norm(p[i]-anchor[i%4])] for i in xrange(0,N)])
    n = random.randn(N,1)*0.1
    measure = y + n
    xe = zeros((N,11))
    xe[0,2] = 0.27
    xe[0,6] = 1
    
    accuracy = 100
    for pp in [0.02*(i**2) for i in xrange(1,9)]:
        for pvxy in [0.01*(i**2) for i in xrange(1,10)]:
            for pvz in [100*(i**2) for i in xrange(1,5)]:
                print pp,pvxy,pvz
                Q[ 0:3,  0:3] =   pp*eye(3)#*10
                Q[ 3:7,  3:7] =  0.01*eye(4)#*10
                Q[ 7:9, 7:9] =  pvxy*eye(2)#/2
                Q[ 9,9] =  pvz#/2
                Q[10,10]      =  0.000000001
                uwb = UWBLocation(1.0/100*num) 
                uwb.setQ(Q)
                timer = sstimer()
                timer.start()
                for i in xrange(0, N-1):
                    xe[i+1], tem = uwb.locate(xe[i], Q, 1.0/100*num, measure[i,0], anchor[i%4], q[i], a[i], r[i])
                
                error = linalg.norm(xe[:,0:3]-p)
                if accuracy > error:
                    accuracy = error
                    opp = pp
                    opvxy = pvxy
                    opvz = pvz
                
                print "Accuracy:", linalg.norm(xe[:,0:3]-p)
                print "Time: ", timer.end()/1000
                
    print opp, opvxy, opvz