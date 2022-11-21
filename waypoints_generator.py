import numpy as np
import matplotlib.pyplot as plt


WAYPOINTS_FILE = 'waypoints/waypoints.txt'

class WaypointGenerator():
    def __init__(self, init_point, end_point, mode):
            self.x_init = init_point[0]
            self.y_init = init_point[1]
            self.x_end = end_point[0]
            self.y_end = end_point[1]

            self.x_displacement = np.abs(self.x_end - self.x_init)
            self.y_displacement = np.abs(self.y_end - self.y_init)

            self.mode = mode			
                
    def generate_speed(self, mode, samples):
        
        x = np.linspace(0,self.x_displacement,samples)

        if mode=='change':
                T = self.x_displacement
                factor = 0.5
                A = factor*2
                B = factor*1
                f = 1/T
                d = T/2

                g = A*np.cos(2*np.pi*f*(x+d))+A
                h = B*np.cos(4*np.pi*f*(x+d))-B

                # Desired speeds
                v = g - h 
        else :
                v = 2*np.ones_like(x)
        return x, v

    def generate_points(self, mode, samples):
        x = np.linspace(self.x_end,self.x_init,samples)
        if mode=='change':
            n = self.x_displacement/2
            S = self.y_displacement
            y = -S/(1+np.exp(-(x-self.x_end-n))) + S + self.y_init
        else:
            y = self.y_init*np.ones_like(x)

        return x,y

    def save_file(self, x, y, v):

        with open(WAYPOINTS_FILE,"w") as f:    
                f.write("")

        x = np.flip(x)
        y = np.flip(y)
        for i in range(1,len(x),1):
                current_x = x[i]
                current_y = y[i]
                current_speed = v[i]
                with open(WAYPOINTS_FILE,"a") as f:    
                        f.write("{}, {}, {}\n".format(current_x, current_y, current_speed))
        
    def generate(self, samples):
        self.x,self.y = self.generate_points(self.mode, samples)
        self.x_v, self.v =self.generate_speed(self.mode, samples)
        self.save_file(self.x,self.y,self.v)

        return self.x, self.y, self.x_v, self.v

    def plot(self):
        x =np.flip(self.x)
        y =np.flip(self.y)
        Ax = [x[0]]
        Ay = [y[0]]

        Bx = [x[-1]]
        By = [y[-1]]

        fig = plt.figure()
        fig.suptitle(r'$Puntos\, de\, trayectoria$')


        ax1 = fig.add_subplot(2,1,1)
        ax1.plot(self.x, self.y, 'r-')

        ax1.plot(Ax,Ay,marker="o", markersize=7, markeredgecolor="red", markerfacecolor="red")
        ax1.annotate('A', (x[0], y[0]+0.5))
        ax1.plot(Bx,By,marker="o", markersize=7, markeredgecolor="blue", markerfacecolor="blue")
        ax1.annotate('B', (x[-1], y[-1]+0.5))

        ax1.invert_xaxis()

        # ax1.set_ybound(129,132.8)

        ax1.set_xlabel("posición x (m)")
        ax1.set_ylabel("posición y (m)")

        ax2 = fig.add_subplot(2,1,2)

        ax2.plot(self.x_v, self.v, 'b-')

        ax2.set_xlabel("desplazamiento (m)")
        ax2.set_ylabel("velocidad (m/s)")

        plt.show()


def main():
    # x_init = 269.347
    # y_init = 129.49

    # x_end = 248.5
    # y_end = 132.5

    distance = 10

    x_init = 269.340
    y_init = 129.490

    x_end = x_init - distance
    y_end = 133

    init_point = [x_init, y_init]
    end_point = [x_end, y_end]

    wpg = WaypointGenerator(init_point, end_point, 'change')
    wpg.generate(100)

    wpg.plot()

main()