import numpy as np
import matplotlib.pyplot as plt
import math




class inter_union(object):
    
    def __init__(self, antecedent_obj, input_obj,disc_no):
    
        self.antecedent=antecedent_obj
        self.input=input_obj
        domain=self.T1_get_Domain()
        self._discrets= np.linspace(domain[0], domain[1], disc_no)
    
    
    def T1_get_Domain(self):
        
        a= min(self.antecedent.interval[0], self.input.interval[0])
        b= max(self.antecedent.interval[1], self.input.interval[1])
        
        return((a,b))
    
        
    
    def T1_intersection_union(self):
                
        intersection_MFs=[]
        union_MFs=[]
        antecedent_degree_s=[]
        input_degree_s=[]

        
        for x in self._discrets:
            a=self.antecedent.get_degree(x)
            b=self.input.get_degree(x)
            antecedent_degree_s.append(a)
            input_degree_s.append(b)
            intersection_MFs.append(min(a,b))
            union_MFs.append(max(a,b))

        
        self._intersection_MFs= np.asarray(intersection_MFs)
        self._union_MFs= np.asarray(union_MFs)
        self._antecedent_degree_s= np.asarray(antecedent_degree_s)
        self._input_degree_s= np.asarray(input_degree_s)
        
        
    @property
    def get_discrete(self):
        return(self._discrets)

    @property
    def intersection(self):
        return self._intersection_MFs
    
    @property
    def union(self):
        return self._union_MFs
    
    @property
    def antecedent_degree_s(self):
        return self._antecedent_degree_s
    
    @property
    def input_degree_s(self):
        return self._input_degree_s
    


        
    def return_firing_stregth(self,technique):
        
        if (technique=="standard"):
            self.T1_intersection_union()
            return(max(self.intersection))
        
        elif(technique=="cen_NS"):
            self.T1_intersection_union()
            centroid_of_intersection=self.centroid_calculation(self.get_discrete, self.intersection)
            if(centroid_of_intersection==0):
                return 0.0
            index=np.searchsorted(self.get_discrete, centroid_of_intersection)
            return(self.intersection[index])
            
        elif(technique=="sim_NS"):
            self.T1_intersection_union()
            return(sum(self.intersection)/float( sum(self.union)) )
        
            
        elif(technique=="sub_NS"):
            self.T1_intersection_union()
            sum_of_intersection=sum(self.intersection)
            sum_of_input=sum(self.input_degree_s)
            return (sum_of_intersection/float(sum_of_input) )
        
                
                
        
    def centroid_calculation(self,disc,membershipdegrees):     
        denominator = 0.0
        numerator = 0.0
        
        numerator = np.dot(disc , membershipdegrees)
        denominator = sum(membershipdegrees)
        
        if not denominator == 0:
            return(numerator / denominator)
        else:
            return (0.0)
            #raise ValueError("centroid denominator=0")    
    
        
