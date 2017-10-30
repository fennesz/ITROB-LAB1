from appJar import gui

class VisionGui:
    def __init__(self):
        self.app = gui()
        self.app.addLabel("title", "Hej")

    def execute(self):
        self.app.go()

if __name__ == '__main__':
    vsgui = VisionGui()
    vsgui.execute()
