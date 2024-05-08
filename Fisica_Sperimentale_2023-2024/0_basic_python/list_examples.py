a = 5
# this is not executed

b = [4,5,3]

b[1] = 2

c = [2,'a',3]

c.append('b')

index = 0 
for index in [0,1,2]: #range(3):
    print (c[index])

for element in c:
    print(element) 



