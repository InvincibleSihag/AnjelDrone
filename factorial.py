import time   

final_list = []    

class GFL:
      
    
    def add(self, a, b):
        return a + b
    def sub(self, a, b):
        return a - b

def factorial(n):   

    time.sleep(.1)   

    factorial = 1   

    for i in range (1,n+1):   

        factorial = factorial * i   

    return factorial   
def sum_factorial():  

    for i in range(5):   

        final_list.append(factorial(i))    

    result=sum(final_list)    

    #print("Final SUM is {}".format(result)) 

    return result

 

sum_factorial()  
