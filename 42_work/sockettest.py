import socket
import time as t

host = 'localhost' #string
port = 10101 #int, check w/ 42 inpcmd.txt

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    print('socket created successfully')

    s.connect(('localhost', 10101))
    print('youre connected! now what...')

    #buffer = ''

    while True:
        t.sleep(1)

        #get state data
        data = s.recv(16384)
        
        if data:
            #buffer += data
            data = data.decode('utf-8')
            #print(type(data))
            print(data)
            #-----------------------parse&process data---------------------

            time = []
            Nsc = 0
            orb_posN = []
            orb_velN = []
            pos_R = []
            vel_R = []
            angvel = []
            QBN = []
            sunvec = []
            magvec = []
            angmom = []

            from datetime import datetime

            data = data.split('\n')

            del data[-1]

            for i, mnem in enumerate(data):
                mnem = mnem.split('  ')
                data[i] = mnem

            for group in data:
                for i in range(len(group)):
                    if group[i] == 'TIME':
                        timestr = group[1]
                        tempo = timestr.split(':')
                        timestr = tempo[0]+':'+tempo[1]
                        #strip seconds so that format is right
                        seconds = tempo[-1].split('.')
                        microsecond = int(seconds[-1])
                        second = int(seconds[0])
                        #ensure that date format is zero-padded, 24-hour
                        time = datetime.strptime(timestr, '%Y-%j-%H:%M')
                        time = time.replace(second=second, microsecond=microsecond)
                        print(f'time: {time}')
                        
                    #if group[i] == 'SC':
                        #process SC number? not needed yet
                        
                    if group[i] == 'ORBPOS_N':
                        orb_posN = group[1].split(' ')
                        orb_posN = [float(comp_i) for comp_i in orb_posN]
                        print(f'orbital position GCI: {orb_posN}')
                        
                    if group[i] == 'ORBVEL_N':
                        orb_velN = group[1].split(' ')
                        orb_velN = [float(comp_i) for comp_i in orb_velN]
                        print(f'orbital velocity GCI: {orb_velN}')
                        
                    if group[i] == 'POS_R':
                        pos_R = group[1].split(' ')
                        pos_R = [float(comp_i) for comp_i in pos_R]
                        print(f'position in R: {pos_R}')
                        
                    if group[i] == 'VEL_R':
                        vel_R = group[1].split(' ')
                        vel_R = [float(comp_i) for comp_i in vel_R]
                        print(f'velocity in R: {vel_R}')
                        
                    if group[i] == 'ANGVEL':
                        angvel = group[1].split(' ')
                        angvel = [float(comp_i) for comp_i in angvel]
                        print(f'angular velocity: {angvel}')
                        
                    if group[i] == 'QBN':
                        qbn = group[1].split(' ')
                        qbn = [float(comp_i) for comp_i in qbn]
                        print(f'quaternion from B to GCI: {qbn}')
                        
                    if group[i] == 'SUNVEC':
                        sunvec = group[1].split(' ')
                        sunvec = [float(comp_i) for comp_i in sunvec]
                        print(f'sun vector: {sunvec}')
                        
                    if group[i] == 'MAGVEC':
                        magvec = group[1].split(' ')
                        magvec = [float(comp_i) for comp_i in magvec]
                        print(f'magnetic field vector: {magvec}')
                        
                    if group[i] == 'ANGMOM':
                        angmom = group[1].split(' ')
                        angmom = [float(comp_i) for comp_i in angmom]
                        print(f'angular momentum: {angmom}')

        #alright, got data in as variables. now what? need to read C functions to process the sensor data
        #then we get commanded attitude, then send the actual commands.

        


            
        else:
            break

        #send command
        s.sendall(b'hey man')    #note: sendall sends ALL data, b is bytes string

    print('done')
