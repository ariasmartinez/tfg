from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pandas
from matplotlib.colors import ListedColormap
import math
import numpy as np

import decimal
import os

folder = 'titopaco'

#files = ['./seventh_try/no_force.csv', './seventh_try/force.csv', './seventh_try/no_force_no_cursor.csv', './seventh_try/force_no_cursor.csv']
#files = ['./mano_izq_celia/no_force.csv', './mano_izq_celia/force.csv', './mano_izq_celia/no_force_no_cursor.csv', './mano_izq_celia/force_no_cursor.csv']
#files = [folder+'/no_force.csv', folder+'/force.csv', folder+'/no_force_no_cursor.csv', folder+'/force_no_cursor.csv']
files = [folder+'/no_force.csv', folder+'/force.csv', folder+'/no_force_no_cursor.csv', folder+'/force_no_cursor.csv']

phase_folder = ['no_force', 'force', 'no_force_no_cursor', 'force_no_cursor']
#files = ['no_force_no_error.csv']
it_0 = 1
it = 81
num_points = 8

def divide(a):
    return a/1000000000


for j in range(4):
        
        if j == 3:
            it = 320
        #for i in range(1, num_points+1):
         #   os.makedirs(folder+'/'+phase_folder[j]+'/'+str(i))
        points = pandas.read_csv(files[j])
      
        differences = []
        differences_total = []
        time_total = []
        time = []
        for i in range (0, num_points):
            differences.append([])
        for i in range (0, num_points):
            time.append([])
        differences_index = []
        for i in range (0, num_points):
            differences_index.append([])
            
        colors = ['black',    'silver',    'red',        'sienna',     'moccasin',          'gold',
                  'orange',   'salmon',    'chartreuse', 'green',      'mediumspringgreen', 'lightseagreen',
                  'darkcyan', 'royalblue', 'blue',       'blueviolet', 'purple',            'fuchsia',
                  'pink',     'tan',       'olivedrab',  'tomato',     'yellow',            'turquoise']
        final_points_x = [2,1.4142,0,-1.4142,-2,-1.4142,0,1.4142]
        final_points_y = [0,1.4142,2,1.4142,0,-1.4142,-2,-1.4142]
        
        #final_points_x = [1,0.7071,0,-0.7071,-1,-0.7071,0,0.7071]
        #final_points_y = [0,0.7071,1,0.7071,0,-0.7071,-1,-0.7071]
        #final_points_x = [1,-1,0,0]
        #final_points_y = [0,0,1,-1]
      
        #final_points_x  = [44.4938,31.4619, 0.000000194,-31.4619,-44.4938 ,-31.4619 , 0.000000194,31.4619 ]
        #final_points_y = [23.75,55.2119,68.2438 ,55.2119, 23.75 ,-7.7119 ,-20.7438,-7.7119 ]
        decimal.getcontext().prec = 3
        
        #x =  list(map(decimal.Decimal, points['x'].values))
        
      
        x = points['x'].values
            
        y = list(map(float, points['y'].values))
 
        color = points['color'].values
        target = points['target'].values
        
      
        
        for i in range(it_0, it):
            print(i)
              
            fig = plt.figure()
            ax = fig.add_subplot(111) 
           
            points_selected = points[points.color == i]
            x_selected = points_selected['x'].values
            #print(points_selected)
            #print(x_selected)
            y_selected = points_selected['y'].values
           
            #print(y_selected)
            color_selected = points_selected['color'].values
            target_selected = points_selected['target'].values
            time_selected = points_selected['time'].values
            

            time_selected = list(map(divide, time_selected))
            
         
            #print(y)
            #print(time_selected[-1])
            #print(x_selected)
            
            ax.scatter(x_selected, y_selected) 
            ax.scatter(x_selected[-1], y_selected[-1], c='yellow') 
            ax.scatter(final_points_x, final_points_y, c='black', s=160)
            path = folder+'/'+phase_folder[j]+'/'+str(int(target_selected[0]))+'/'+str(i)+'.png'
            #plt.savefig(path)
            plt.show()
            
            #print(x_selected[-1])
            #print(y_selected[-1])

            tar = int(target_selected[0])-1
           
            dif_x = abs(x_selected[-1]-final_points_x[int(target_selected[0])-1]) 
           
            
            dif_y = abs(y_selected[-1]-final_points_y[int(target_selected[0])-1])
            
            
            #differences.append((math.sqrt((abs(x[-1]-final_points_x[target[0]-1])-0.15)**2+(abs(y[-1]-final_points_y[target[0]-1])**2)-0.15)))
            dist_global = math.sqrt(dif_x**2+dif_y**2)
            if dist_global > 0.06:
                dist_rel = dist_global - 0.06
            else:
                dist_rel = 0
            differences[tar].append(dist_rel)
            
            differences_index[tar].append(i)
            differences_total.append(dist_rel)
            time[tar].append(time_selected[-1])
            time_total.append(time_selected[-1])
        #ax.scatter(x, y, c=color, cmap=ListedColormap(colors))        
        #ax.scatter(final_points_x, final_points_y, c='black', s=1500)
        #numbers = np.arange(it_0, it)  # the label locations; needs to start at 1, not 0
        
        #plt.show()
        
        #plt.show()
        for i in range (0,len(differences)):
            ax_2 = fig.add_subplot(111) 
            
            if (len(differences[i]) > 0):
               
                ax_2.stem = plt.stem(differences_index[i], differences[i])
                plt.ylim(0, 2.1) # hacer que el max sea el max de los valores
                path = folder+'/'+phase_folder[j]+'/'+str(i+1)+'/resumen.png'
                #plt.savefig(path)
                plt.show()
                
        ax_3 = fig.add_subplot(111) 
        ax_3.stem = plt.stem(np.arange(it_0,it), differences_total)
        plt.ylim(0, 2.1)
        print(differences_total)
        path = folder+'/'+phase_folder[j]+'/resumen_total.png'
        #plt.savefig(path)
        plt.show()

        for i in range (0,len(time)):
            ax_4 = fig.add_subplot(111) 
            
            if (len(time[i]) > 0):
               
                
                ax_4.stem = plt.stem(differences_index[i], time[i])
                
                plt.ylim(1, 2.2) # hacer que el max sea el max de los valores
                path = folder+'/'+phase_folder[j]+'/'+str(i+1)+'/time.png'
                plt.savefig(path)
                plt.show()
                
        ax_5 = fig.add_subplot(111) 
        ax_5.stem = plt.stem(np.arange(it_0,it), time_total)
        plt.ylim(1, 2.2)
        path = folder+'/'+phase_folder[j]+'/time.png'
        plt.savefig(path)
        plt.show()
        it=it+80
        it_0=it_0+80

#27.7657, 51.1042

# 28.225, 50.9948