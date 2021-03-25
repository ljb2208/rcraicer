from kivy.app import App
from kivy.uix.widget import Widget
from kivy.graphics import *

class TrackBuilderWidget(Widget):
    def redraw(self, args, args2):
        print("redraw")
        with self.canvas:
            # Add a red color
            Color(1., 0, 0)

            # Add a rectangle
            Rectangle(pos=(10, 10), size=(500, 500))


class TrackBuilderApp(App):
    def build(self):
        widget = TrackBuilderWidget()
        widget.bind(pos=widget.redraw, size=widget.redraw)
        return widget

if __name__ == '__main__':
    TrackBuilderApp().run()