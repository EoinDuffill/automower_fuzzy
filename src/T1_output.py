
import numpy as np

class output(object):

    def __init__(self, mean, step, interval, max_fs):
        
        self._mean=mean
        self._step=step
        self._interval= interval
        self._max_fs = max_fs
        self.list_of_x = np.linspace(self._interval[0], self._interval[1], 100)
      
    @property
    def interval(self):
        return self._interval
    
    @property
    def mean(self):
        return self._mean
    
    @property
    def max_fs(self):
        return self._max_fs

    def set_max_fs(self,fs):
        self._max_fs=fs
        
        
class T1_Triangular_output(output):

    def __init__(self, a, b, c, max_fs):
        output.__init__(self, b, (c - a)/4.0, (a, c), max_fs)

    def get_degree(self, x):
        
        left=self._interval[0]
        right=self._interval[1]
         
        if(x <= left or x>= right):
            degree=0.0
        elif(x ==self._mean):
            degree = self._max_fs
        elif(x < self._mean ):
            degree = min( ( (x - left) / (self._mean - left) ) , self._max_fs )       
        elif(x > self._mean ):
            degree = min( (  (right - x) / (right - self._mean) ) , self._max_fs )
            
        return(degree)

    def get_mf_degrees(self):
        list_of_mf=[]
        for i in self.list_of_x:
            list_of_mf.append(self.get_degree(i))
        list_of_mf=np.asarray(list_of_mf)
        return(self.list_of_x, list_of_mf)
    
class T1_RightShoulder_output(output):

    def __init__(self, a, b, c, max_fs):
        output.__init__(self, b, (c - a) / 4.0, (a, c), max_fs)

    def get_degree(self, x):
        if(x<self.interval[0] or x>self.interval[1]):
            return(min(0.0,self._max_fs))
        elif(x>=self.mean and x<=self.interval[1] ):
            return(min(1.0,self._max_fs))
        elif(x<self.mean and x>=self.interval[0]):
            return (min ( ((x-self.interval[0])/float(self.mean - self.interval[0])),self._max_fs) )
        else:
            raise ValueError("Something wrong with x in Right Shoulder.")

    def get_mf_degrees(self):
        list_of_mf=[]
        for i in self.list_of_x:
            list_of_mf.append(self.get_degree(i))
        list_of_mf=np.asarray(list_of_mf)
        return(self.list_of_x, list_of_mf)


class T1_LeftShoulder_output(output):

    def __init__(self, a, b, c, max_fs):
        output.__init__(self, b, (c - a) / 4.0, (a, c), max_fs)

    def get_degree(self, x):
        if( x>self.interval[1]):
            return(min(0.0,self._max_fs))
        elif(x<=self.mean):
            return(min(1.0,self._max_fs))
        elif(x>self.mean and x<=self.interval[1]):
            return (min( ((self.interval[1]-x)/float(self.interval[1]-self.mean)),self._max_fs)  )
        else:
            raise ValueError("Something wrong with x in Left Shoulder.") 

    def get_mf_degrees(self):
        list_of_mf=[]
        for i in self.list_of_x:
            list_of_mf.append(self.get_degree(i))
        list_of_mf=np.asarray(list_of_mf)
        return(self.list_of_x, list_of_mf)