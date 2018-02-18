from tkinter import *
from tkinter.filedialog import askopenfilename
from PIL import Image, ImageTk
import serial
from tkinter import messagebox, font
from math import *

L1 = 0.3070
L2 = 0.4550
L3  = 0.4550
LA = 0.0300
LB = 0.0420

def degreesToRad(degrees):
    return (float)((degrees) * pi / 180)
def radToDegrees(rad):
    return (float)((rad) * 180 / pi)

class structure:
    def __init__(self):
        self.a = 0

class Interface():
    x = 0
    y = 0
    x_arm = 0
    z_arm = 0
    line = 0
    dot = 0
    zero_offset_x = 0
    zero_offset_y = 350.0
    pix_to_meter_y = 0.0
    pix_to_meter_x = 0.0
    total_pix_y = 537#699
    total_pix_x = 961#1250

    def __init__(self):
        self.canvas = None
        self.root = Tk()
        self.root.title('RHA Serial Interface')
        self.exit_button = Button(self.root, text='Cerrar', command=quit).pack(side=BOTTOM, pady=(10,10))
        self.ser = serial.Serial('/dev/ttyACM1', 921600, timeout=1)
        self.setControllingArea()


    def setControllingArea(self):
        self.frame_control_entries = Frame(self.root, borderwidth=2, relief="raised")
        custom_font = font.Font(weight='bold')
        self.frame_info = Frame(self.frame_control_entries, borderwidth=2, relief="raised")
        self.etiq1 = Label(self.frame_info, text="Información general:", font=custom_font).pack(side=TOP)
        self.tinfo = Text(self.frame_info, width=40, height=18)
        self.tinfo.pack(side=TOP)
        self.frame_info.pack(side=TOP, pady=(10,10))

        #self.articular_pos_goal = structure()
        #self.articular_pos_goal.q1 = self.articular_pos_goal.q2 = self.articular_pos_goal.q3 = 0
        #self.cartesian_pos_goal = structure()
        #self.cartesian_pos_goal.x = self.cartesian_pos_goal.y = self.cartesian_pos_goal.z = 0

        frame_articular_coords = Frame(self.frame_control_entries, borderwidth=2, relief="raised", width=40, height=10)
        self.etiq_articular = Label(frame_articular_coords, text="Coordenadas Articulares:", font=custom_font).grid(row=0,padx=(10,0), pady=(4,4), columnspan=2)
        self.etiq_q1 = self.makeentry(frame_articular_coords, "Q1:", 1, 0, 6) #, textvariable=self.articular_pos_goal.q1)
        self.etiq_q2 = self.makeentry(frame_articular_coords, "Q2:", 2, 0, 6) #, textvariable=self.articular_pos_goal.q2)
        self.etiq_q3 = self.makeentry(frame_articular_coords, "Q3:", 3, 0, 6) #, textvariable=self.articular_pos_goal.q3)
        self.boton_aceptar_articular = Button(frame_articular_coords, text="Send", width=30, command=self.sendArticular).grid(row=4, padx=(10,10), pady=(4,4), columnspan=2)
        frame_articular_coords.pack(side=TOP, padx=(10,10), pady=(10,10))

        frame_cartesian_coords = Frame(self.frame_control_entries, borderwidth=2, relief="raised", width=40, height=10)
        self.etiq_articular = Label(frame_cartesian_coords, text="Coordenadas Cartesianas:", font=custom_font).grid(row=0,padx=(10,0), pady=(4,4), columnspan=2)
        self.etiq_x = self.makeentry(frame_cartesian_coords, "X:", 1, 0, 6) #, textvariable=self.cartesian_pos_goal.x)
        self.etiq_y = self.makeentry(frame_cartesian_coords, "Y:", 2, 0, 6) #, textvariable=self.cartesian_pos_goal.y)
        self.etiq_z = self.makeentry(frame_cartesian_coords, "Z:", 3, 0, 6) #, textvariable=self.cartesian_pos_goal.z)
        self.boton_aceptar_cartesian = Button(frame_cartesian_coords, text="Send", width=30, command=self.sendCartesian).grid(row=4, padx=(10,10), pady=(4,4), columnspan=2)
        frame_cartesian_coords.pack(side=TOP, padx=(10,10), pady=(10,10))

        self.frame_control_entries.pack(side=LEFT, pady=(10,10))
    def makeentry(self, parent, caption, position_row, position_column, width=None, **options):
        Label(parent, text=caption).grid(row=position_row, column=position_column)
        entry = Entry(parent, **options)
        if width:
            entry.config(width=width)
        entry.grid(row=position_row, column=position_column+1, padx=(0,10))
        return entry

    def sendArticular(self):
        articular_pos = structure()
        articular_pos.q1 = float(self.etiq_q1.get()) #self.articular_pos_goal.q1
        articular_pos.q2 = float(self.etiq_q2.get()) #self.articular_pos_goal.q2
        articular_pos.q3 = float(self.etiq_q3.get()) #self.articular_pos_goal.q3
        self.sendSerialInfo(articular_pos)

    def sendCartesian(self):
        cartesian_pos = structure()
        articular_pos = structure()
        cartesian_pos.x = float(self.etiq_x.get()) #self.cartesian_pos_goal.x
        cartesian_pos.y = float(self.etiq_y.get()) #self.cartesian_pos_goal.y
        cartesian_pos.z = float(self.etiq_z.get()) #self.cartesian_pos_goal.z
        articular_pos = self.inverseKinematics(cartesian_pos)
        self.sendSerialInfo(articular_pos)

    def main(self):

        #setting up a tkinter canvas with scrollbars
        #frame = Frame(root, bd=2, relief=SUNKEN)
        self.frame = Frame(self.root, bd=2, relief=SUNKEN, width=self.total_pix_x, height = self.total_pix_y)
        self.frame.grid_rowconfigure(1, weight=1)
        self.frame.grid_columnconfigure(0, weight=1)
        #xscroll = Scrollbar(self.frame, orient=HORIZONTAL)
        #xscroll.grid(row=1, column=0, sticky=E+W)
        #yscroll = Scrollbar(self.frame)
        #yscroll.grid(row=1, column=1, sticky=N+S)
        custom_font = font.Font(weight='bold')
        self.etiqueta_rango = Label(self.frame, text="Rango de movimiento posible:", font=custom_font).grid(row=0,padx=(10,0), pady=(4,4), columnspan=2)
        self.canvas = Canvas(self.frame, bd=0)#, xscrollcommand=xscroll.set, yscrollcommand=yscroll.set)
        self.canvas.grid(row=1, column=0, sticky=N+S+E+W, pady=(4,4))
        #xscroll.config(command=self.canvas.xview)
        #yscroll.config(command=self.canvas.yview)
        self.frame.pack(side=RIGHT,fill=BOTH,expand=1)

        #adding the image
        #File = askopenfilename(parent=root, initialdir="C:/",title='Choose an image.')
        #img = ImageTk.PhotoImage(Image.open(File))
        img = Image.open("rango_movimiento.png")
        img = img.resize((self.total_pix_x, self.total_pix_y), Image.ANTIALIAS)
        self.pix_to_meter_x = self.total_pix_x/1
        self.pix_to_meter_y = self.total_pix_y/1.2
        img = ImageTk.PhotoImage(img)
        self.canvas.create_image(0,0,image=img,anchor="nw")
        #self.canvas.config(scrollregion=self.canvas.bbox(ALL))

        #mouseclick event
        self.canvas.bind("<ButtonPress-1>", self.on_button_press)
        self.canvas.bind("<ButtonRelease-1>", self.on_button_release)
        #canvas.bind("<Button 1>",printcoords)
        self.root.mainloop()

    def on_button_press(self, event):
        self.x = event.x
        self.y = event.y

    def on_button_release(self, event):
        python_green = "#476042"
        x0,y0 = (self.x_arm, self.z_arm)
        x1,y1 = (event.x, event.y)
        self.canvas.delete(self.line)
        self.line = self.canvas.create_line(x0, y0, x1, y1)
        self.send_x = (event.x-self.zero_offset_x)/self.pix_to_meter_x
        self.send_z = -(event.y-self.zero_offset_y)/self.pix_to_meter_y

        print ("Send goal position: (" + str(x) + ", " + str(y) + ")")
        cartesian_pos = structure()
        cartesian_pos.x  = self.send_x
        cartesian_pos.y = 0
        cartesian_pos.z = self.send_z
        articular_pos = inverseKinematics(cartesian_pos)
        sendSerialInfo(articular_pos)

    def sendSerialInfo(self, articular_pos):
        checksum = (~(6 + 3 + int(articular_pos.q1) + int(articular_pos.q2) + int(articular_pos.q3)) & 0xF) #in python is needed to mask the operation

        print ("Goal sent: "+str(articular_pos.q1)+" "+str(articular_pos.q2)+" "+str(articular_pos.q3))
        values = bytearray([int(255),int(255),int(6),int(3),int(articular_pos.q1),int(articular_pos.q2),int(articular_pos.q3),checksum]) #255 is 0xFF in hex
        # length = 6 bytes (not header)
        # 3 -> articular_position goal
        #print (values)
        self.ser.write(values)

    def forwardKinematics (self, articular_pos):
        cartesian_pos = structure()
        cartesian_pos.x  = LA + L2*cos(pi/2-degreesToRad(180-articular_pos.y)) + L2*cos(degreesToRad(180-articular_pos.z)-pi/2)
        cartesian_pos.y = 0
        cartesian_pos.z = LB + L2*sin(pi/2-degreesToRad(180-articular_pos.y)) - L2*sin(degreesToRad(180-articular_pos.z)-pi/2) + L1
        return cartesian_pos

    def inverseKinematics (self, cartesian_pos):
        articular_pos = structure()
        xa = cartesian_pos.x - LA
        za = cartesian_pos.z - L1 - LB

        articular_pos.q1 = 0
        articular_pos.q2 = 180-radToDegrees(pi/2 - (acos(sqrt(xa*xa + za*za)/(2*L2)) + atan2(za,xa)));
        articular_pos.q3 = 180-radToDegrees(pi - (acos(sqrt(xa*xa + za*za)/(2*L2)) + atan2(za,xa)) - acos((L2*L2+L2*L2-sqrt(xa*xa + za*za)*sqrt(xa*xa + za*za))/(2*L2*L2)) + pi/2);
        return articular_pos

    def printInfo(self):
        self.tinfo.delete("1.0", END) # deletes previous info

        articular_pos = structure()
        articular_pos.x = int(self.joint_first.pose)
        articular_pos.y = int(self.joint_second.pose)
        articular_pos.z = int(self.joint_third.pose)

        cartesian_pos = structure()
        cartesian_pos = self.forwardKinematics(articular_pos)

        text_info = "\n"
        text_info += " Primera Articulación: " + "\n"
        text_info += "   Posición: " + str(self.joint_first.pose) + "\n"
        text_info += "   Velocidad: " + str(self.joint_first.speed) + "\t\t\t dir: " + {True: "UP", False: "DOWN"}[self.joint_first.speed_dir is True] + "\n"
        text_info += "   Torque Aplicado: " + str(self.joint_first.torque) + "\t\t\t dir: " + {True: "CW", False: "CCW"}[self.joint_first.torque_dir is True] + "\n"
        text_info += "\n"
        text_info += " Segunda Articulación: " + "\n"
        text_info += "   Posición: " + str(self.joint_second.pose) + "\n"
        text_info += "   Velocidad: " + str(self.joint_second.speed) + "\t\t\t dir: " + {True: "UP", False: "DOWN"}[self.joint_second.speed_dir is True] + "\n"
        text_info += "   Torque Aplicado: " + str(self.joint_second.torque) + "\t\t\t dir: " + {True: "CW", False: "CCW"}[self.joint_second.torque_dir is True] + "\n"
        text_info += "\n"
        text_info += " Tercera Articulación: " + "\n"
        text_info += "   Posición: " + str(self.joint_third.pose) + "\n"
        text_info += "   Velocidad: " + str(self.joint_third.speed) + "\t\t\t dir: " + {True: "UP", False: "DOWN"}[self.joint_third.speed_dir is True] + "\n"
        text_info += "   Torque Aplicado: " + str(self.joint_third.torque) + "\t\t\t dir: " + {True: "CW", False: "CCW"}[self.joint_third.torque_dir is True] + "\n"
        text_info += "\n"
        text_info += " Posición extremo (m): (" + '%.2f'%(cartesian_pos.x) + "," + '%.2f'%(cartesian_pos.y) + "," + '%.2f'%(cartesian_pos.z) + ")" + "\n"

        self.tinfo.insert("1.0", text_info)

    def printErrorBanner(self):
        print ("blabla")

    def serialListener(self):
        while self.ser.in_waiting:
            #print ("Reading info")
            if bytes(self.ser.read()[0]) == bytes(0xFF) and bytes(self.ser.read()[0]) == bytes(0xFF):
                #print ("Package accepted")
                length = self.ser.read()
                value = self.ser.read(length[0]-1)
                if value[0] == 0:  # 0 means an UPDATE_INFO package
                    self.joint_first = structure()
                    self.joint_first.pose = int(value[1])
                    self.joint_first.speed = (int(value[2]) | (int(value[3]) << 8))
                    self.joint_first.speed_dir = ((self.joint_first.speed & 0x0400) >> 10)
                    self.joint_first.speed = self.joint_first.speed & ~0x0400
                    self.joint_first.torque = (int(value[4]) | (int(value[5]) << 8))
                    self.joint_first.torque_dir = ((self.joint_first.torque & 0x0400) >> 10)
                    self.joint_first.torque = self.joint_first.torque & ~0x0400
                    self.joint_second = structure()
                    self.joint_second.pose = int(value[6])
                    self.joint_second.speed = (int(value[7]) | (int(value[8]) << 8))
                    self.joint_second.speed_dir = ((self.joint_second.speed & 0x0400) >> 10)
                    self.joint_second.speed = self.joint_second.speed & ~0x0400
                    self.joint_second.torque = (int(value[9]) | (int(value[10]) << 8))
                    self.joint_second.torque_dir = ((self.joint_second.torque & 0x0400) >> 10)
                    self.joint_second.torque = self.joint_second.torque & ~0x0400
                    self.joint_third = structure()
                    self.joint_third.pose = int(value[11])
                    self.joint_third.speed = (int(value[12]) | (int(value[13]) << 8))
                    self.joint_third.speed_dir = ((self.joint_third.speed & 0x0400) >> 10)
                    self.joint_third.speed = self.joint_third.speed & ~0x0400
                    self.joint_third.torque = (int(value[14]) | (int(value[15]) << 8))
                    self.joint_third.torque_dir = ((self.joint_third.torque & 0x0400) >> 10)
                    self.joint_third.torque = self.joint_third.torque & ~0x0400
                    self.printInfo()
                elif value[0] == 1: #packet contains error information
                    error = ""
                    print ("Error ocurred")
                    if value[1]!= 0:
                        error += "Error en servo: "
                    if value[1] & 0x01:
                        error += "\n - servo articulacion 1"
                    if value[1] & 0x02:
                        error += "\n - servo articulacion 2"
                    if value[1] & 0x04:
                        error += "\n - servo articulacion 3"
                    if value[2]!= 0:
                        error = "\nError en articulación: "
                    if value[2] & 0x01:
                        error += "\n - articulacion 1"
                    if value[2] & 0x02:
                        error += "\n - articulacion 2"
                    if value[2] & 0x04:
                        error += "\n - articulacion 3"
                    messagebox.showerror("Error",error)
                    sys.exit(0)
        self.root.after(10,self.serialListener)


if __name__ == "__main__":
    interface = Interface()
    interface.serialListener()
    interface.main()
    interface.ser.close()
