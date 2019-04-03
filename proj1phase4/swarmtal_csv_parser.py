import numpy as np
import scipy
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import math
import scipy
import sympy as sp
# sys.path.append('C:\\Users\\plane\\Dropbox\\FlightDevelop\\pyAircraftIden')
# from AircraftIden import FreqIdenSIMO, TransferFunctionFit,TransferFunctionModel,TransferFunctionParamModel
# from AircraftIden import FreqIdenSIMO, TransferFunctionFit

plt.rcParams.update({'font.size': 16})
plt.rcParams["font.family"] = "Monospaced"
import matplotlib as mpl
mpl.rc('lines', linewidth=2)

plt.rc('figure', figsize=(20, 10))
def parse_csv_data(csv_path, lt=0, rt=1000000):
    data =  np.genfromtxt(csv_path, delimiter=',')   
    l = 0
    r = len(data[:,0]) - 1
    while data[l, 0] < lt:
        l += 1
        
    while data[r, 0] > rt:
        r -= 1
    
    print(f"Find time {lt}:{rt}s with {l}:{r}")
    
    ans = {}

    """
    ts : 0
    ctrl_mode 1
    posx, posy, posz 2:5
    velx, vely, velz 5:8
    att r p y 8:11
    pos_sp x y z 11:14
    vel_sp x y z 14:17
    acc_sp x y z 17:20
    attout rpy 20:23
    thrsp 24
    """
    ans["ts"] = data[l:r,0]
    ans["ctrl_mode"] = data[l:r,1]
    ans['pos'] = data[l:r,2:5]
    ans['vel'] = data[l:r,5:8]
    ans['rpy'] = data[l:r,8:11]
    ans['pos_sp'] = data[l:r,11:14]
    ans['vel_sp'] = data[l:r,14:17]
    ans['acc_sp'] = data[l:r,17:20]
    ans["rpy_sp"] = data[l:r,20:23]
    ans["thr_sp"] = data[l:r,23]
    ans['rpy_fc'] = data[l:r,24:26]
    return ans

def anaylze_csv(dataname, l=0, r=100000, plot=True):
    csv_data = parse_csv_data(dataname, l, r)
    
    def maskps(data):
        return np.ma.masked_where(csv_data["ctrl_mode"] != 2, data)
    
    _t = csv_data["ts"]
    if plot:
        plt.title('Mode')
        plt.plot(_t, csv_data["ctrl_mode"])
        plt.grid()

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax = fig.gca(projection='3d')
        ax.plot(csv_data['pos'][:,0], csv_data['pos'][:,1], csv_data['pos'][:,2], label="POS")
        ax.plot(csv_data['pos_sp'][:,0], csv_data['pos_sp'][:,1], csv_data['pos_sp'][:,2], label="POS_SP")


        quiver_step = 20
        plt.quiver(csv_data['pos'][::quiver_step,0], csv_data['pos'][::quiver_step,1], csv_data['pos'][::quiver_step,2],
                   csv_data['vel'][::quiver_step,0], csv_data['vel'][::quiver_step,1], csv_data['vel'][::quiver_step,2],
                  color="orange",length=0.3)

        plt.grid()
        plt.legend()

        plt.figure("Position Tracking")
    #     plt.title("Position Tracking")
        plt.plot(_t, csv_data['pos'][:,0], label="x")
        plt.plot(_t, csv_data['pos'][:,1], label="y")
        plt.plot(_t, csv_data['pos'][:,2], label="z")
        plt.plot(_t, maskps(csv_data['pos_sp'][:,0]), label="spx")
        plt.plot(_t, maskps(csv_data['pos_sp'][:,1]), label="spy")
        plt.plot(_t, maskps(csv_data['pos_sp'][:,2]), label="spz")

        plt.xlabel("Time(s)")
        plt.ylabel("Position (m)")
        plt.legend(loc='lower right')
        plt.grid(which="both")
        #plt.show()

        plt.rc('figure', figsize=(20, 10))
        fig = plt.figure("PosXYXZ")
        ax = fig.add_subplot(121)
    #     plt.title("Position XZ Following")
        ax.plot(csv_data['pos'][:,0], -csv_data['pos'][:,2], label='real')
        ax.plot(csv_data['pos_sp'][:,0], -csv_data['pos_sp'][:,2], label='sp')
        ax.legend(loc='lower right')
        ax.grid(which="both")
        plt.xlabel("X (m)")    
        plt.ylabel("Z (m)")

        ax = fig.add_subplot(122)
    #     plt.title("Position XY Following")
        ax.plot(csv_data['pos'][:,0], csv_data['pos'][:,1], label='real')
        ax.plot(csv_data['pos_sp'][:,0], csv_data['pos_sp'][:,1], label='sp')
        ax.legend(loc='best')
        ax.grid(which="both")
        plt.xlabel("X (m)")    
        plt.ylabel("Y (m)")


        plt.figure("PosErr")
    #     plt.title("Position Tracking Error")
        plt.plot(_t, maskps(csv_data['pos'][:,0] - csv_data['pos_sp'][:,0]), label="x")
        plt.plot(_t, maskps(csv_data['pos'][:,1] - csv_data['pos_sp'][:,1]), label="y")
        plt.plot(_t, maskps(csv_data['pos'][:,2] - csv_data['pos_sp'][:,2]), label="z")
        plt.xlabel("Time(s)")
        plt.ylabel("Error (m)")
        plt.legend(loc='upper right')
        plt.grid(which="both")


        plt.figure("Velocity")
    #     plt.title("Velocity")
        ax = plt.subplot(311)
        plt.plot(_t, csv_data['vel'][:,0], label="velX_actual")
        plt.plot(_t, csv_data['vel_sp'][:,0], label="velX_cmd")
    #     plt.plot(_t, csv_data['vel_des'][:,0], label="des_x")
        plt.setp(ax.get_xticklabels(), visible=False)
        plt.legend(loc='upper right')
        plt.grid(which="both")
        plt.ylabel("(m/s)")

        ax = plt.subplot(312)
        plt.plot(_t, csv_data['vel'][:,1], label="velY_actual")
        plt.plot(_t, csv_data['vel_sp'][:,1], label="velY_cmd")
    #     plt.plot(_t, csv_data['vel_des'][:,1], label="des_y")
        plt.setp(ax.get_xticklabels(), visible=False)
        plt.ylabel("(m/s)")

        plt.legend(loc='upper right')
        plt.grid(which="both")

        ax = plt.subplot(313)
        plt.plot(_t, csv_data['vel'][:,2], label="velZ_actual")
        plt.plot(_t, csv_data['vel_sp'][:,2], label="velZ_cmd")
    #     plt.plot(_t, csv_data['vel_des'][:,2], label="des_z")
        plt.ylabel("(m/s)")
        plt.xlabel("Time(s)")


        plt.legend(loc='upper right')
        plt.grid(which="both")

        plt.figure("PRY")
        ax = plt.subplot(311)
    #     plt.title("Pitch")
        plt.plot(_t, csv_data['rpy_sp'][:,0]*57.296, label="roll_cmd")
        plt.plot(_t, csv_data['rpy_fc'][:,0]*57.296, label="roll_fc")
        plt.plot(_t, csv_data['rpy'][:,0]*57.296, label="roll_actual")
#         plt.ylim(-10, 10) 
        plt.setp(ax.get_xticklabels(), visible=False)
        plt.ylabel("Roll (°)")

        plt.legend(loc='upper right')
        plt.grid(which="both")


        ax = plt.subplot(312)
    #     plt.title("Roll")
        plt.plot(_t, csv_data['rpy_sp'][:,1]*57.296, label="pitch_cmd")
        plt.plot(_t, -csv_data['rpy_fc'][:,1]*57.296, label="pitch_fc")
        plt.plot(_t, -csv_data['rpy'][:,1]*57.296, label="pitch_actual")
        plt.setp(ax.get_xticklabels(), visible=False)
        plt.ylabel("Pitch (°)")
#         plt.ylim(-10, 10) 
        plt.legend(loc='upper right')
        plt.grid(which="both")


        ax = plt.subplot(313)
    #     plt.title("Yaw")
    #     plt.plot(_t, -csv_data['rpy_fc'][:,2]*57.296, label="yaw_fc")
        plt.plot(_t, csv_data['rpy'][:,2]*57.296, label="yaw_actual")
        plt.plot(_t, csv_data['rpy_sp'][:,2]*57.296, label="yaw_cmd")
        plt.ylabel("Yaw (°)")
        plt.legend(loc='upper right')
        plt.grid(which="both")
        plt.xlabel("Time(s)")
        plt.setp(ax.get_xticklabels(), visible=True)


        plt.rc('figure', figsize=(15, 10))
        plt.figure("Acc")
        plt.subplot(221)
        plt.plot(_t, csv_data['acc_sp'][:,0] , label="$a_x$_cmd")
        plt.legend(loc='upper right')
        plt.grid(which="both")
        plt.setp(ax.get_xticklabels(), visible=False)
        plt.ylabel("($m/s^2$)")

        plt.subplot(222)
    #     plt.title("Acc Y")        

        plt.plot(_t, csv_data['acc_sp'][:,1], label="$a_y$_cmd")
        plt.setp(ax.get_xticklabels(), visible=False)
        plt.ylabel("($m/s^2$)")

        plt.legend(loc='upper right')
        plt.grid(which="both")

        plt.subplot(223)
    #     plt.title("Acc Z")        

        plt.plot(_t, csv_data['acc_sp'][:,2], label="$a_z$_cmd")
        plt.ylabel("($m/s^2$)")

    #     plt.ylim(-2, 5)

        plt.legend(loc='upper right')
        plt.grid(which="both")
        plt.xlabel("Time (s)")

        """
        plt.subplot(224)
        plt.plot(_t, csv_data["acc"][:,2], 'o', label="ABX_actual")
        plt.plot(_t, csv_data['pry_abx_cmd'][:,3], label="ABX_cmd")
        plt.legend(loc='upper right')
        plt.grid(which="both")
        plt.xlabel("Time (s)")
        plt.ylabel("($m/s^2$)")
        """

        plt.show()
    
    return csv_data
    

if __name__=="__main__":
    if len(sys.argv)> 1:
        cd = anaylze_csv(sys.argv[1], plot=True)