from tkinter import *
from tkinter.filedialog import askopenfilename
from PIL import Image, ImageTk
import serial


class Interface():
    x = 0
    y = 0
    x_arm = 0
    z_arm = 0
    line = 0
    dot = 0
    zero_offset_x = 39
    zero_offset_y = 346
    pixel_decimeter_height = 68.25 #217.15pixel/dm
    pixel_decimeter_width = 148.90 #217.15pixel/dm
    def __init__(self):
        self.canvas = None
        self.root = Tk()
        #TODO:self.ser = serial.Serial('/dev/ttyACM0', 921600, timeout=1)

    def main(self):

        #setting up a tkinter canvas with scrollbars
        #frame = Frame(root, bd=2, relief=SUNKEN)
        frame = Frame(self.root, bd=2, relief=SUNKEN, width=938, height = 550)
        frame.grid_rowconfigure(0, weight=1)
        frame.grid_columnconfigure(0, weight=1)
        xscroll = Scrollbar(frame, orient=HORIZONTAL)
        xscroll.grid(row=1, column=0, sticky=E+W)
        yscroll = Scrollbar(frame)
        yscroll.grid(row=0, column=1, sticky=N+S)
        self.canvas = Canvas(frame, bd=0, xscrollcommand=xscroll.set, yscrollcommand=yscroll.set)
        self.canvas.grid(row=0, column=0, sticky=N+S+E+W)
        #xscroll.config(command=self.canvas.xview)
        #yscroll.config(command=self.canvas.yview)
        frame.pack(fill=BOTH,expand=1)

        #adding the image
        #File = askopenfilename(parent=root, initialdir="C:/",title='Choose an image.')
        #img = ImageTk.PhotoImage(Image.open(File))
        img = Image.open("rango_movimiento.png")
        img = img.resize((1250, 699), Image.ANTIALIAS)
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
        print ((event.x-self.zero_offset_x)*0.001,(event.y-self.zero_offset_y)*0.001)

    def serialListener(self):
        """TODO:while self.ser.in_waiting:
                value = self.ser.read(6) #read devuelve un vector
                if value[0] == 0xFF and value[1] == 0xFF and value[5] == ~(bytes(value[2])+bytes(value[3])+bytes(value[4])):
                    self.x_arm = value[2] + self.zero_offset_x
                    self.y_arm = value[4] + self.zero_offset_y
                    x1, y1 = (self.x_arm - 2), (self.y_arm - 2)
                    x2, y2 = (self.x_arm + 2), (self.y_arm + 2)
                    self.canvas.delete(self.dot)
                    self.dot = self.canvas.create_oval(x1, y1, x2, y2, fill=python_green)

        self.root.after(10,self.serialListener)"""


if __name__ == "__main__":
    interface = Interface()
    interface.serialListener()
    interface.main()
    interface.ser.close()
