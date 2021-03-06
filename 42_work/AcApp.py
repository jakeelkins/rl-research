import socket
import io
import time as t

host = 'localhost' #string
port = 10101 #int, check w/ 42 inpcmd.txt

#socket init
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('socket created successfully')

s.connect((host, port))

print('youre connected!')

def ReadFromSocket(socket):

    #get state data
    data = s.recv(16384)

    if data:
        done = False
        while not done:
            #buffer += data

            #decoder
            #what we do now is split at '[EOF]' marker to get away from bad stuff
            decoded = False
            while not decoded:
                try:
                    datalist = data.split(b'[EOF]') #should split into 2

                    data_io = io.BytesIO(datalist[0])

                    wrapper = io.TextIOWrapper(data_io, encoding='utf-8')

                    read_data = wrapper.read()

                    decoded = True
                except Exception as decoded_err:
                    print(f'DECODE ERROR: {decoded_err}')
                    

                #way I used to have it, throws occasional errors
                #try:
                    #data = data.decode('utf-8') #original, where error was thrown
                    #decoded = True
                    #break
                #except Exception as utf8err:
                    #print(utf8err)
                #data = data.decode('unicode_escape') ####THIS WORKS
                #decoded = True
                #data = data.decode('unicode_escape').encode('utf-8') #kinda weird
            
            #print(type(read_data)) #str
            #print(read_data)

            #-----------------------parse&process data---------------------
            t.sleep(0.01) #sim the process comp time


            #get to EOF or 16383
            done = True
        
        #send ack
        ackmsg = b"Ack\n"

        socket.send(ackmsg)  #maybe sendall here?
    return read_data

def WriteToSocket(socket):

    #construct message of commands
    t.sleep(0.01) #placeholder

    #msg = b"SC[0].AC.Whl[0].Tcmd = [ -1.366773e-03]\n[EOF]\n\n"

    msg = b'SC[0].AC.svb = [ -6.170506e-01 1.428959e-02 -7.867937e-01]\nSC[0].AC.bvb = [ 1.806000e-05 -1.384000e-05 2.396000e-05]\nSC[0].AC.Hvb = [ -6.888402e-02 -3.483317e-01 1.092438e-01]\nSC[0].AC.G[0].Cmd.Ang = [ -2.476523e+01 0.000000e+00 0.000000e+00]\nSC[0].AC.Whl[0].Tcmd = [ 4.000000e+01]\nSC[0].AC.Whl[1].Tcmd = [ 0.000000e+00]\nSC[0].AC.Whl[2].Tcmd = [ 0.000000e+00]\nSC[0].AC.Whl[3].Tcmd = [ 0.000000e+00]\nSC[0].AC.MTB[0].Mcmd = [ 1.000000e+02]\nSC[0].AC.MTB[1].Mcmd = [ 0.000000e+00]\nSC[0].AC.MTB[2].Mcmd = [ 0.000000e+00]\nSC[0].AC.Cmd.Ang = [ 0.000000e+00 0.000000e+00 0.000000e+00]\n[EOF]\n\n'
    #send command
    socket.send(msg) #sendall?

    #wait for ack
    socket.recv(8)

#def VariableProcessing(in_string):
    


'''
#init all variables here that variable processor writes to, do later
class SC:

    def __init__(self, number):
        self.number = number
        self.AC = AC()
        self.truth = truth()

    class AC:
        def __init__(self):
            self.

'''



while True:
    inc_data = ReadFromSocket(s)

    print(inc_data)

    in_string = in_string.strip()
    in_string = in_string.split('\n')

    for item in in_string:
        item = item.split(' ')
        if item[0] == 'TIME':
            time = item[1]
        if item[0] == 'SC[0].PosR':
            PosR = [0, 0, 0]
            PosR[-1] = float(item[-1].strip(' []'))
            PosR[-2] = float(item[-2].strip(' []'))
            PosR[-3] = float(item[-3].strip(' []'))
        if item[0] == 'SC[0].VelR':
            VelR = [0, 0, 0]
            VelR[-1] = float(item[-1].strip(' []'))
            VelR[-2] = float(item[-2].strip(' []'))
            VelR[-3] = float(item[-3].strip(' []'))        
        if item[0] == 'SC[0].svb':
            svb = [0, 0, 0]
            svb[-1] = float(item[-1].strip(' []'))
            svb[-2] = float(item[-2].strip(' []'))
            svb[-3] = float(item[-3].strip(' []'))
        if item[0] == 'SC[0].bvb':
            bvb = [0, 0, 0]
            bvb[-1] = float(item[-1].strip(' []'))
            bvb[-2] = float(item[-2].strip(' []'))
            bvb[-3] = float(item[-3].strip(' []'))
        if item[0] == 'SC[0].Hvb':
            Hvb = [0, 0, 0]
            Hvb[-1] = float(item[-1].strip(' []'))
            Hvb[-2] = float(item[-2].strip(' []'))
            Hvb[-3] = float(item[-3].strip(' []'))
        if item[0] == 'SC[0].AC.G[0].Ang':
            ac_g0_ang = [0, 0, 0]
            ac_g0_ang[-1] = float(item[-1].strip(' []'))
            ac_g0_ang[-2] = float(item[-2].strip(' []'))
            ac_g0_ang[-3] = float(item[-3].strip(' []'))
        if item[0] == 'SC[0].AC.Gyro[0].Rate':
            ac_gyro0_rate = 0
            ac_gyro0_rate = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.Gyro[1].Rate':
            ac_gyro1_rate = 0
            ac_gyro1_rate = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.Gyro[2].Rate':
            ac_gyro2_rate = 0
            ac_gyro2_rate = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.MAG[0].Field':
            ac_mag0_field = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.MAG[1].Field':
            ac_mag1_field = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.MAG[2].Field':
            ac_mag2_field = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[0].Valid':
            ac_css0_valid = 0
            ac_css0_valid = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[0].Illum':
            ac_css0_illum = 0
            ac_css0_illum = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[1].Valid':
            ac_css1_valid = 0
            ac_css1_valid = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[1].Illum':
            ac_css1_illum = 0
            ac_css1_illum = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[2].Valid':
            ac_css2_valid = 0
            ac_css2_valid = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[2].Illum':
            ac_css2_illum = 0
            ac_css2_illum = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[3].Valid':
            ac_css3_valid = 0
            ac_css3_valid = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[3].Illum':
            ac_css3_illum = 0
            ac_css3_illum = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[4].Valid':
            ac_css4_valid = 0
            ac_css4_valid = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[4].Illum':
            ac_css4_illum = 0
            ac_css4_illum = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[5].Valid':
            ac_css5_valid = 0
            ac_css5_valid = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[5].Illum':
            ac_css5_illum = 0
            ac_css5_illum = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[6].Valid':
            ac_css6_valid = 0
            ac_css6_valid = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[6].Illum':
            ac_css6_illum = 0
            ac_css6_illum = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[7].Valid':
            ac_css7_valid = 0
            ac_css7_valid = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.CSS[7].Illum':
            ac_css7_illum = 0
            ac_css7_illum = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.FSS[0].Valid':
            ac_fss0_valid = 0
            ac_fss0_valid = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.FSS[0].SunAng':
            ac_fss0_sunang = 0
            ac_fss0_sunang = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.ST[0].Valid':
            ac_st0_valid = 0
            ac_st0_valid = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.ST[0].qn':
            ac_st0_qn = [0, 0, 0, 0]
            ac_st0_qn[-1] = float(item[-1].strip(' []'))
            ac_st0_qn[-2] = float(item[-2].strip(' []'))
            ac_st0_qn[-3] = float(item[-3].strip(' []'))
            ac_st0_qn[-4] = float(item[-4].strip(' []'))
        if item[0] == 'SC[0].AC.GPS[0].Valid':
            ac_gps0_valid = 0
            ac_gps0_valid = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.GPS[0].Rollover':
            ac_gps0_rollover = 0
            ac_gps0_rollover = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.GPS[0].Week':
            ac_gps0_week = 0
            ac_gps0_week = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.GPS[0].Sec':
            ac_gps0_sec = 0
            ac_gps0_sec = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.GPS[0].PosN':
            ac_gps_posn = [0, 0, 0]
            ac_gps_posn[-1] = float(item[-1].strip(' []'))
            ac_gps_posn[-2] = float(item[-2].strip(' []'))
            ac_gps_posn[-3] = float(item[-3].strip(' []'))
        if item[0] == 'SC[0].AC.GPS[0].VelN':
            ac_gps_veln = [0, 0, 0]
            ac_gps_veln[-1] = float(item[-1].strip(' []'))
            ac_gps_veln[-2] = float(item[-2].strip(' []'))
            ac_gps_veln[-3] = float(item[-3].strip(' []'))
        if item[0] == 'SC[0].AC.GPS[0].PosW':
            ac_gps_posw = [0, 0, 0]
            ac_gps_posw[-1] = float(item[-1].strip(' []'))
            ac_gps_posw[-2] = float(item[-2].strip(' []'))
            ac_gps_posw[-3] = float(item[-3].strip(' []'))
        if item[0] == 'SC[0].AC.GPS[0].VelW':
            ac_gps_velw = [0, 0, 0]
            ac_gps_velw[-1] = float(item[-1].strip(' []'))
            ac_gps_velw[-2] = float(item[-2].strip(' []'))
            ac_gps_velw[-3] = float(item[-3].strip(' []'))
        if item[0] == 'SC[0].AC.GPS[0].Lng':
            ac_gps0_lng = 0
            ac_gps0_lng = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.GPS[0].Lat':
            ac_gps0_lat = 0
            ac_gps0_lat = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.GPS[0].Alt':
            ac_gps0_alt = 0
            ac_gps0_alt = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.Whl[0].H':
            ac_whl0_h = 0
            ac_whl0_h = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.Whl[1].H':
            ac_whl1_h = 0
            ac_whl1_h = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.Whl[2].H':
            ac_whl2_h = 0
            ac_whl2_h = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].AC.Whl[3].H':
            ac_whl3_h = 0
            ac_whl3_h = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].B[0].wn':
            b0_wn = [0, 0, 0]
            b0_wn[-1] = float(item[-1].strip(' []'))
            b0_wn[-2] = float(item[-2].strip(' []'))
            b0_wn[-3] = float(item[-3].strip(' []'))
        if item[0] == 'SC[0].B[0].qn':
            b0_qn = [0, 0, 0, 0]
            b0_qn[-1] = float(item[-1].strip(' []'))
            b0_qn[-2] = float(item[-2].strip(' []'))
            b0_qn[-3] = float(item[-3].strip(' []'))
            b0_qn[-4] = float(item[-4].strip(' []'))
        if item[0] == 'SC[0].B[1].wn':
            b1_wn = [0, 0, 0]
            b1_wn[-1] = float(item[-1].strip(' []'))
            b1_wn[-2] = float(item[-2].strip(' []'))
            b1_wn[-3] = float(item[-3].strip(' []'))
        if item[0] == 'SC[0].B[1].qn':
            b1_qn = [0, 0, 0, 0]
            b1_qn[-1] = float(item[-1].strip(' []'))
            b1_qn[-2] = float(item[-2].strip(' []'))
            b1_qn[-3] = float(item[-3].strip(' []'))
            b1_qn[-4] = float(item[-4].strip(' []'))
        if item[0] == 'SC[0].Whl[0].H':
            whl0_h = 0
            whl0_h = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].Whl[1].H':
            whl1_h = 0
            whl1_h = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].Whl[2].H':
            whl2_h = 0
            whl2_h = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].Whl[3].H':
            whl3_h = 0
            whl3_h = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].Gyro[0].TrueRate':
            gyro0_truerate = 0
            gyro0_truerate = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].Gyro[1].TrueRate':
            gyro1_truerate = 0
            gyro1_truerate = float(item[-1].strip(' []'))
        if item[0] == 'SC[0].Gyro[2].TrueRate':
            gyro2_truerate = 0
            gyro2_truerate = float(item[-1].strip(' []'))
        if item[0] == 'World[3].PosH':
            world3_posh = [0, 0, 0]
            world3_posh[-1] = float(item[-1].strip(' []'))
            world3_posh[-2] = float(item[-2].strip(' []'))
            world3_posh[-3] = float(item[-3].strip(' []'))
        if item[0] == 'World[3].eph.PosN':
            world3_eph_posn = [0, 0, 0]
            world3_eph_posn[-1] = float(item[-1].strip(' []'))
            world3_eph_posn[-2] = float(item[-2].strip(' []'))
            world3_eph_posn[-3] = float(item[-3].strip(' []'))
        if item[0] == 'World[3].eph.VelN':
            world3_eph_veln = [0, 0, 0]
            world3_eph_veln[-1] = float(item[-1].strip(' []'))
            world3_eph_veln[-2] = float(item[-2].strip(' []'))
            world3_eph_veln[-3] = float(item[-3].strip(' []'))
        if item[0] == 'World[10].PosH':
            world10_posh = [0, 0, 0]
            world10_posh[-1] = float(item[-1].strip(' []'))
            world10_posh[-2] = float(item[-2].strip(' []'))
            world10_posh[-3] = float(item[-3].strip(' []'))
        if item[0] == 'World[10].eph.PosN':
            world10_eph_posn = [0, 0, 0]
            world10_eph_posn[-1] = float(item[-1].strip(' []'))
            world10_eph_posn[-2] = float(item[-2].strip(' []'))
            world10_eph_posn[-3] = float(item[-3].strip(' []'))
        if item[0] == 'World[10].eph.VelN':
            world10_eph_veln = [0, 0, 0]
            world10_eph_veln[-1] = float(item[-1].strip(' []'))
            world10_eph_veln[-2] = float(item[-2].strip(' []'))
            world10_eph_veln[-3] = float(item[-3].strip(' []'))
        if item[0] == 'Orb[0].PosN':
            orb0_posn = [0, 0, 0]
            orb0_posn[-1] = float(item[-1].strip(' []'))
            orb0_posn[-2] = float(item[-2].strip(' []'))
            orb0_posn[-3] = float(item[-3].strip(' []'))
        if item[0] == 'Orb[0].VelN':
            orb0_veln = [0, 0, 0]
            orb0_veln[-1] = float(item[-1].strip(' []'))
            orb0_veln[-2] = float(item[-2].strip(' []'))
            orb0_veln[-3] = float(item[-3].strip(' []'))





    WriteToSocket(s)


