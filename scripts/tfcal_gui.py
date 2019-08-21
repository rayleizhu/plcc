import Tkinter

# TODO: in development

class Calculator(object):
    def __init__(self):
        self.root = Tkinter.Tk()
        self.root.title('Transformation Calculator')
        self.inputbox1=Tkinter.Entry(self.root, width=50)
        self.inputbox2=Tkinter.Entry(self.root, width=50)
        self.displaybox=Tkinter.Listbox(self.root, width=50)
        self.cal_button=Tkinter.Button(self.root,
                                       command=self.calc,
                                       text='calculate')
        self.inputbox1.pack()
        self.inputbox2.pack()
        self.displaybox.pack()
        self.cal_button.pack()
    
    def calc(self):
        tf1 = self.inputbox1.get()
        tf2 = self.inputbox2.get()

        info = ['tf1:{}'.format(tf1), 'tf2:{}'.format(tf2)]
        for item in info:
            self.displaybox.insert(0, item)


cal = Calculator()
Tkinter.mainloop()