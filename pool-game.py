import tkinter
import time
import random

CANVAS_WIDTH = 400
CANVAS_HEIGHT = 700


BALL_SIZE = 30
RADIUS = BALL_SIZE/2
POCKET_SIZE = 50


def main():
    canvas = make_canvas(CANVAS_WIDTH, CANVAS_WIDTH, 'POOL')
    table = canvas.create_rectangle(0,0, CANVAS_WIDTH, CANVAS_HEIGHT, fill= 'green', outline = 'green')
    
    pockets = make_pockets(canvas)
    
    x0 = CANVAS_WIDTH / 2 - RADIUS
    
    y0 = CANVAS_HEIGHT - BALL_SIZE * 4
    
    ball_0 = canvas.create_oval(x0,y0, (x0+BALL_SIZE), (y0+BALL_SIZE), fill='white')
    
    x1 = CANVAS_WIDTH / 2 - RADIUS
    y1 = BALL_SIZE * 6
    
    ball_1 = canvas.create_oval(x1, y1, (x1+BALL_SIZE), (y1+BALL_SIZE), fill=color())
    
    dx = 0.3
    dy = -7
    
    dx0 = 2.5
    dy0 = -5
    
    # Atualize as posições da bola branca para perceber as colisões, tanto na bola 0 (bola teste), quanto nas paredes  
    
    dx1 = -7
    dy1 = -8
    
    
    while not hit_ball_1(canvas, ball_0, ball_1):
        canvas.move(ball_0, dx, dy)
        if hit_left_wall(canvas, ball_0) or hit_right_wall(canvas, ball_0):
            dx *= -1
        
        if hit_top_wall(canvas, ball_0) or hit_bottom_wall(canvas, ball_0):
            dy *= -1
            
        
    while True:
        canvas.move(ball_0, dx0, dy0)
        canvas.move(ball_1, dx1, dy1)
        
        if hit_left_wall(canvas, ball_0) or hit_right_wall(canvas, ball_0):
            dx0 *= -1
            
        if hit_top_wall(canvas, ball_0) or hit_bottom_wall(canvas, ball_0):
            dy0 *= -1
            
        if hit_left_wall(canvas, ball_1) or hit_right_wall(canvas, ball_1):
            dx1 *= -1
            
        if hit_top_wall(canvas, ball_1) or hit_bottom_wall(canvas, ball_1):
            dy1 *= -1
            
        if hit_pockets(canvas, ball_1):
            canvas.delete(ball_1)
            break
        canvas.update()
        time.sleep(1 / 25.)
    canvas.mainloop()
    
def make_pockets(canvas):
    x = CANVAS_WIDTH - POCKET_SIZE #start pixels for pockets 2, 4, 6 on the right wall
    y = CANVAS_HEIGHT - POCKET_SIZE #start pixels for pockets 5 and 6 on the bottom wall
    y1 = CANVAS_HEIGHT / 2 - POCKET_SIZE / 2 #start pixels for pockets 3 and 4 in the center
    y2 = CANVAS_HEIGHT / 2 + POCKET_SIZE / 2 #end pixels for pockets 3 and 4 in the center
    
    pocket_1 = canvas.create_oval(0, 0, POCKET_SIZE, POCKET_SIZE, fill='black')
    
    pocket_2 = canvas.create_oval(x, 0, CANVAS_WIDTH, POCKET_SIZE, fill='black')
    
    pocket_3 = canvas.create_oval(0, y1, POCKET_SIZE, y2, fill='black')
    
    pocket_4 = canvas.create_oval(x, y1, CANVAS_WIDTH, y2, fill='black')
    
    pocket_5 = canvas.create_oval(0, y, POCKET_SIZE, CANVAS_HEIGHT, fill='black')
    
    pocket_6 = canvas.create_oval(x, y, CANVAS_WIDTH, CANVAS_HEIGHT, fill='black')
    
def color():
    return random.choice(['red', 'blue', 'yellow', 'purple', 'brown', 'orange', 'pink', 'gray', 'tan', 'chartreuse'])


def hit_pockets(canvas, object):
    object_coords = canvas.coords(object)
    
    x1 = object_coords[0]
    
    y1 = object_coords[1]
    
    x2 = object_coords[2]
    
    y2 = object_coords[3]
    
    results = canvas.find_overlapping(x1, y1, x2, y2)
    return len(results) > 2


def hit_ball_1(canvas, ball_0, ball_1):
    ball_1_coords = canvas.coords(ball_1)
    
    x1 = ball_1_coords[0]
    
    y1 = ball_1_coords[1]
    
    x2 = ball_1_coords[2]
    
    y2 = ball_1_coords[3]
    
    results = canvas.find_overlapping(x1, y1, x2, y2)
    return len(results) > 2


def hit_left_wall(canvas, object):
    return canvas.coords(object)[0] <= 0

def hit_top_wall(canvas, object):
    return canvas.coords(object)[1] <= 0

def hit_right_wall(canvas, object):
    return canvas.coords(object)[2] >= CANVAS_WIDTH

def hit_bottom_wall(canvas, object):
    return canvas.coords(object)[3] >= CANVAS_HEIGHT

def make_canvas(width, height, title):
    
    top = tkinter.Tk()
    top.minsize(width=width, height=height)
    top.title(title)
    canvas = tkinter.Canvas(top, width=width+1, height=height+1)
    canvas.pack()
    return canvas

if __name__ == '__main__':
    main()