#importing pygmaps 
import pygmaps 
#inport webbrowser to view output
import webbrowser as wb
# list of latitudes 
#29.8648599,77.8965787 Random Initial Coordinates

mymap3 = pygmaps.maps(29.867069, 77.895098, 15) 
a = 29.867069
b = 77.895098
  
for i in range(0,5,1): 
  
    # add a point into a map 
    # 1st argument is latitude 
    # 2nd argument is longitude 
    # 3rd argument is colour of the point showed in thed map 
    # using HTML colour code e.g. 
    # red "# FF0000", Blue "# 0000FF", Green "# 00FF00" 

  	#generate multiple coordinates (FOR Reference)
    mymap3.addpoint(a,b, "# FF0000") 
    b = b+0.0002
    a = a+0.0002
#generate HTML file with required data 
mymap3.draw('pygmap3.html') 
#open the HTML file
wb.open_new('pygmap3.html')
