from steuerung import Fahrzeug
from steuerung import readSequenz

import cocos
from cocos.text import Label
from cocos.sprite import Sprite
from cocos.actions import *
from cocos.director import director
from cocos.layer import Layer, ColorLayer, ScrollingManager, ScrollableLayer
from pyglet.window import key

class Driving(Driver):
    
    def __init__(self, robot):
            super().__init__()
            
            self.robot = robot
            
    def step(self, dt):
        
        # This line might seem pretty complicated, but it's really not at all
        self.target.rotation += (keyboard[key.RIGHT] - keyboard[key.LEFT]) * 100 * dt
        # Essentially what I do here is take the right value minus the left value (remember that moving left is negative)
        # And then I multiply it by 100 so it's more visible, and multiply it by the dt value passed in by the step function
        # Finally, I add it to the rotation of the "target", which would be the sprite we tell to do this action

        # Now I'm going to do something very similar for the sprite's acceleration
        self.target.acceleration = (keyboard[key.UP] - keyboard[key.DOWN]) * 350
        # See if you can figure this out yourself!

        # Next I'm going to make the car stop completely if the space key is held
        if keyboard[key.SPACE]:
            self.target.speed = 0
            # Pretty easy, huh?
        
        # That's basically it!
        # Now we just need to call the original step function to let it do its magic
        super().step(dt)
        
class Mecanum(Layer):
    '''
    robot  - Fahrzeugobjekt
    sprite - Bild
    '''
    #is_event_handler = True

    def __init__(self):
        super(Mecanum, self).__init__()

        #Mecanum wheel robot
        seq = readSequenz('cur.seq')
        print(seq)
        self.robot = Fahrzeug(seq, None, 200, 10)
        self.sprite = Sprite('assets/robot.png')
        self.sprite.position = 700, 50
        self.sprite.scale = 0.5
        self.add(self.sprite)
        self.sprite.do(Driving(self.robot))

        #pressed keys in hud
        self.hud = Label("",
                      font_name = "Helvetica",
                      font_size = 20,
                      color = (0,0,0,255),
                      anchor_x = "left",
                      anchor_y = "center")
        self.hud.position = 5, 20

        #pressed keys
        self.keys_being_pressed = set()
        self.update_text()
        self.add(self.hud)
    #Shows currently pressed keys
    def update_text(self):
        self.hud.element.text = self.robot.status()

    '''def on_key_press(self, key, modifiers):
        
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
        self.update_text()'''
        
#init
director.init(width=1400, height=800, resizable=True)

keyboard = key.KeyStateHandler()
director.window.push_handlers(keyboard)

scene = cocos.scene.Scene()
scene.add(ColorLayer(255,255,255,255))
scene.add(Mecanum())
director.run(scene)