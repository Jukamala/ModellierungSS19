import cocos
from cocos.text import Label
from cocos.layer import Layer, ColorLayer
from cocos import layer
from cocos.sprite import Sprite
from cocos.actions import *
from cocos.director import director
from time import sleep
from pyglet.window.key import symbol_string
        
class Mecanum(ColorLayer):
    
    is_event_handler = True

    def __init__(self):
        super(Mecanum, self).__init__(255, 255, 255, 255)

        #Mecanum wheel robot
        self.sprite = Sprite('assets/robot.png')
        self.sprite.position = 320, 240
        #fade in
        self.sprite.opacity = 0
        self.add(self.sprite, z=1)
        self.sprite.do(FadeIn(0.5))

        #pressed keys in hud
        self.keyhud = Label("Keys: ",
                      font_name = "Helvetica",
                      font_size = 20,
                      anchor_x = "left",
                      anchor_y = "center")
        self.keyhud.position = 0, 20

        #pressed keys
        self.keys_being_pressed = set()
        self.update_text()
        self.add(self.keyhud)
    #Shows currently pressed keys
    def update_text(self):
        key_names = [symbol_string(k) for k in self.keys_being_pressed]
        text_for_label = "Keys: " + ", ".join(key_names)
        self.keyhud.element.text = text_for_label

    def on_key_press(self, key, modifiers):
        
        if symbol_string(key) == 'W':
            self.sprite.do(MoveBy((0,10), 0.2) + MoveBy((0,-10), 0.2))
        if symbol_string(key) == 'A':
            self.sprite.do(MoveBy((-10,0), 0.2))
        if symbol_string(key) == 'S':
            self.sprite.do(MoveBy((0,-10), 0.2))
        if symbol_string(key) == 'D':
            self.sprite.do(MoveBy((10,0), 0.2))
            
        self.keys_being_pressed.add(key)
        self.update_text()
        
    def on_key_release(self, key, modifiers):
        self.keys_being_pressed.remove(key)
        self.update_text()


# I initialize the director and run the layer (this is typical for cocos programs)
director.init()
director.run(cocos.scene.Scene(Mecanum()))