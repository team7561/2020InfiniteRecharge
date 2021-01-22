import tkinter
import time
from PIL import ImageTk, Image

root = tkinter.Tk()
root.geometry("1000x600")

c = tkinter.Canvas(root, height=500, width=1000)

x = 150
y = 50

img = ImageTk.PhotoImage(Image.open("download.jfif"))  
robot= c.create_image(x, y, image=img) 


def field1():
    global x
    global y
    c.update()
    robot= c.create_image(x, y, image=img) 
    c.tag_raise(robot)
    c.update()
    print("Field 1")

def move(shape, x1, y1):
    global x
    global y
    c.move(shape, x1, y1)
    x = x1 + x
    y = y1 + y
    time.sleep(0.05)
    c.update()

field1 = tkinter.Button(root, text="Field 1", command=field1)

c.grid(row=0, column=0)
field1.grid(row=1, column=0)
root.mainloop()